"""
Detection of blocking waits in executor threads.

Design rule of the e-MDB: callbacks never wait blocked. A callback that waits for
something that only another callback of the same executor can deliver blocks
that executor forever (and, even when the answer comes from elsewhere, it stops
every node of the executor meanwhile). Callbacks that need a response must await
it (ServiceClientAsync), and long loops run in their own thread.

Synchronous waits call check_blocking_wait first. If they run in an executor
thread, it logs an error with the stack, once per call site, and the wait goes
on. With the environment variable EMDB_STRICT_BLOCKING_WAITS=1 it raises an
exception instead (for tests: an exception in a callback stops the whole
execution node).

Executor threads are detected in two ways:
- Python executors (SingleThreadedExecutor, MultiThreadedExecutor): the stack of
  the thread contains frames of rclpy/executors.py.
- Executors implemented in C++ (EventsExecutor), whose frames are not visible:
  the thread that spins them must call register_executor_thread() first.
"""

import os
import sys
import threading
import traceback

import rclpy.logging

STRICT_ENV_VARIABLE = "EMDB_STRICT_BLOCKING_WAITS"
RCLPY_EXECUTORS_FILE = os.path.join("rclpy", "executors.py")

_executor_threads = set()  # Idents of the threads registered with register_executor_thread
_reported_sites = set()  # Call sites already reported
_lock = threading.Lock()
_logger = rclpy.logging.get_logger("blocking_wait_check")


def register_executor_thread():
    """
    Registers the current thread as an executor thread. Call it right before
    spinning an executor whose callbacks run in this thread and are not visible
    in the Python stack (EventsExecutor).
    """
    with _lock:
        _executor_threads.add(threading.get_ident())


def in_executor_thread():
    """
    Tells whether the current thread runs executor callbacks.

    :return: True if the thread is registered or runs a Python executor.
    :rtype: bool
    """
    if threading.get_ident() in _executor_threads:
        return True
    frame = sys._getframe(1)
    while frame is not None:
        if frame.f_code.co_filename.endswith(RCLPY_EXECUTORS_FILE):
            return True
        frame = frame.f_back
    return False


def check_blocking_wait(description, skip_files=()):
    """
    Reports a blocking wait that is about to happen in an executor thread.

    :param description: What is going to be waited for, for the report.
    :type description: str
    :param skip_files: Files of the wait implementation, skipped to find the call site.
    :type skip_files: tuple of str
    :raises RuntimeError: If the wait runs in an executor thread and EMDB_STRICT_BLOCKING_WAITS=1.
    """
    if not in_executor_thread():
        return
    stack = traceback.extract_stack()[:-1]
    site = next((f for f in reversed(stack) if f.filename != __file__ and not f.filename.endswith(skip_files)), stack[-1])
    message = (f"Blocking wait in an executor thread: {description}, at {site.filename}:{site.lineno}. "
               "It blocks every callback of this executor until it finishes. Await the response instead "
               "(ServiceClientAsync) or wait in a thread of your own.")
    if os.environ.get(STRICT_ENV_VARIABLE) == "1":
        raise RuntimeError(message)
    with _lock:
        if (site.filename, site.lineno) in _reported_sites:
            return
        _reported_sites.add((site.filename, site.lineno))
    _logger.error(message + " Reported once per call site. Stack:\n" + "".join(traceback.format_list(stack)))

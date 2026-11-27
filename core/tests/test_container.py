"""Tests of the chronological order kept by Container in buffer_index."""

import sys
import types

import numpy as np
import pytest

try:
    import core_interfaces.msg  # noqa: F401
except ImportError:
    # Container only needs the message type to build messages: these tests can run without ROS.
    _msg = types.ModuleType("core_interfaces.msg")
    _msg.Container = type("Container", (), {})
    _interfaces = types.ModuleType("core_interfaces")
    _interfaces.msg = _msg
    sys.modules["core_interfaces"] = _interfaces
    sys.modules["core_interfaces.msg"] = _msg

from core.container import Container  # noqa: E402


def make_container(capacity):
    return Container("test", capacity, "test", labels=["value"])


def push_values(container, values):
    values = np.asarray(values, dtype=np.float64)
    container.push(values.reshape(-1, 1), timestamps=values)


def assert_order(container, expected):
    """Every way of reading the samples must return them from the oldest to the newest."""
    expected = np.asarray(expected, dtype=np.float64)
    all_positions = list(range(len(expected)))
    copy = make_container(container.max_size)
    copy.push(container)

    readings = {
        "read()": container.read().values,
        "read_numpy()": container.read_numpy()[0],
        "read(list)": container.read(all_positions).values,
        "read(slice)": container.read(slice(None)).values,
        "push(container)": copy.read().values,
    }
    for name, values in readings.items():
        assert values.ravel().tolist() == expected.tolist(), name

    # buffer_index gives the position of each valid sample in the chronological order.
    buffer_index = container.data.coords["buffer_index"].values
    values = container.data.values.ravel()
    valid = container.data.coords["valid"].values
    assert values[valid][np.argsort(buffer_index[valid])].tolist() == expected.tolist()
    assert sorted(buffer_index[valid].tolist()) == all_positions


def test_several_samples_wrap_a_partially_filled_container():
    container = make_container(4)
    push_values(container, [1, 2, 3])
    push_values(container, [4, 5, 6])

    assert container.data.coords["buffer_index"].values.tolist() == [2, 3, 0, 1]
    assert_order(container, [3, 4, 5, 6])


@pytest.mark.parametrize("capacity", [3, 5, 100])
def test_single_samples_wrap_a_capacity_that_is_not_a_power_of_two(capacity):
    # 0 - 1 is 2**32 - 1 in uint32, and its remainder is capacity - 1 only for powers of two.
    container = make_container(capacity)
    pushed = list(range(1, 2 * capacity + 2))
    for value in pushed:
        push_values(container, [value])

    assert_order(container, pushed[-capacity:])


@pytest.mark.parametrize("capacity", range(1, 17))
def test_order_after_any_sequence_of_pushes(capacity):
    for first in range(capacity + 1):
        for size in range(1, capacity + 1):
            container = make_container(capacity)
            pushed = list(range(1, first + 3 * size + 1))
            if first:
                push_values(container, pushed[:first])
            for start in range(first, len(pushed), size):
                push_values(container, pushed[start:start + size])

            assert_order(container, pushed[-capacity:])


def test_order_after_clear():
    container = make_container(3)
    push_values(container, [1, 2, 3, 4])
    container.clear()
    push_values(container, [5, 6])
    assert_order(container, [5, 6])
    push_values(container, [7, 8])
    assert_order(container, [6, 7, 8])

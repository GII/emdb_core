from __future__ import annotations
import numpy as np
import xarray as xr
from array import array


from core_interfaces.msg import Container as ContainerMsg

DTYPE_TO_CODE = {
    np.dtype(np.float32): 1,
    np.dtype(np.float64): 2,
    np.dtype(np.int32): 3,
    np.dtype(np.int64): 4,
    np.dtype(np.uint8): 5,
    np.dtype(np.bool_): 6,
}

CODE_TO_DTYPE = {v: k for k, v in DTYPE_TO_CODE.items()}


def _format_debug_table(columns: list[str], rows: list[list[str]]) -> str:
    """Simple monospace table formatter for __repr__ output."""
    if not columns:
        return ""

    widths = [len(str(c)) for c in columns]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))

    def fmt_row(row_vals: list[str]) -> str:
        return " | ".join(str(v).ljust(widths[i]) for i, v in enumerate(row_vals))

    sep = "-+-".join("-" * w for w in widths)
    lines = [fmt_row(columns), sep]
    lines.extend(fmt_row(r) for r in rows)
    return "\n".join(lines)


class Container:
    @property
    def name(self) -> str | None:
        return self.data.name

    @name.setter
    def name(self, value: str | None) -> None:
        self.data.name = value

    @property
    def size(self) -> int:
        valid = self._valid_cache
        return int(np.count_nonzero(valid))
    
    @property
    def feature_labels(self) -> list[str]:
        return self._feature_labels_cache.tolist()

    def __init__(self, name, max_size: int, container_type: str, data_type=np.float64, labels: list = None, attrs: dict = None) -> Container:
        self.feature_dim = f"features"
        self.container_type = container_type
        feature_labels = labels if labels is not None else []
        shape = (max_size, len(feature_labels))

        coords = {
            self.feature_dim: feature_labels,
            "timestamp": ("sample", np.zeros(max_size, dtype=np.float64)),
            "buffer_index": ("sample", np.full(max_size, -1).astype(np.uint32)),
            "valid": ("sample", np.zeros(max_size, dtype=bool)),
        }
        data = np.full(shape, np.nan, dtype=data_type)

        self.data = xr.DataArray(
            data=data,
            dims=["sample", self.feature_dim],
            coords=coords,
            name=name,
            attrs=attrs if attrs is not None else {},
        )
        self.msg = ContainerMsg()

        # Caches for access to frequently used data and metadata.
        self._feature_index_cache: dict[tuple[tuple[str, ...], tuple[str, ...]], np.ndarray] = {}
        self._attrs_pairs_cache = None
        self._attrs_written = False

        # Ring metadata for O(1) slot resolution.
        self._write_cursor = 0
        self._n_valid = 0

        self._refresh_cache()

    def __len__(self):
        return self.size

    def _refresh_cache(self) -> None:
        self._valid_cache = self.data.coords["valid"].values
        self._buffer_index_cache = self.data.coords["buffer_index"].values
        self._timestamp_cache = self.data.coords["timestamp"].values
        self._feature_cache = self.data.values
        self._feature_labels_cache = self.data.coords[self.feature_dim].values
        self.max_size = int(self.data.sizes["sample"])

    def _get_feature_take_indices(
        self,
        src_labels: tuple[str, ...],
        dst_labels: tuple[str, ...],
    ) -> np.ndarray:
        cache_key = (src_labels, dst_labels)
        cached = self._feature_index_cache.get(cache_key)
        if cached is not None:
            return cached

        src_pos = {label: i for i, label in enumerate(src_labels)}
        try:
            take_idx = np.fromiter(
                (src_pos[label] for label in dst_labels),
                dtype=np.int64,
                count=len(dst_labels),
            )
        except KeyError as exc:
            missing = str(exc.args[0])
            raise ValueError(f"Missing destination feature in source container: {missing}") from exc

        self._feature_index_cache[cache_key] = take_idx
        return take_idx
    
    def _is_ring_step_sequence(self, slots: np.ndarray) -> bool:
        if slots.size <= 1:
            return True
        diffs = (slots[1:] - slots[:-1]) % self.max_size
        return bool(np.all(diffs == 1))
    
    def _ordered_all_valid_slots(self) -> np.ndarray:
        n = int(self._n_valid)
        if n == 0:
            return np.empty(0, dtype=np.int64)

        start = (self._write_cursor - n) % self.max_size
        end = start + n

        if end <= self.max_size:
            return np.arange(start, end, dtype=np.int64)

        first = np.arange(start, self.max_size, dtype=np.int64)
        second = np.arange(0, end % self.max_size, dtype=np.int64)
        return np.concatenate((first, second))

    def _resolve_slot(self, index: int, by_buffer_index: bool = False, require_valid: bool = True) -> int:
        if by_buffer_index:
            matches = np.where(self._buffer_index_cache == index)[0]
            if matches.size == 0:
                raise IndexError(f"buffer_index {index} not found")
            slot = int(matches[0])
        else:
            slot = int(index)

        if slot < 0 or slot >= self.max_size:
            raise IndexError(f"slot {slot} out of range [0, {self.max_size - 1}]")

        if require_valid and not bool(self._valid_cache[slot]):
            raise IndexError(f"slot {slot} has no valid data")

        return slot
    
    def _update_buffer_index_fallback(self, slots: np.ndarray) -> np.ndarray:
        # Original robust behavior for non-ring-ordered writes.
        buffer_indexes = self._buffer_index_cache
        valid = self._valid_cache

        old_valid_slots = np.flatnonzero(valid)
        if old_valid_slots.size > 0:
            old_order = np.argsort(buffer_indexes[old_valid_slots], kind="stable")
            old_valid_slots = old_valid_slots[old_order]

        keep_old = old_valid_slots[~np.isin(old_valid_slots, slots)]

        new_order = np.concatenate([keep_old, slots])
        n_valid = new_order.size

        buffer_indexes[:] = -1
        valid[:] = False
        valid[new_order] = True
        buffer_indexes[new_order] = np.arange(n_valid, dtype=np.int64)

        # Reconstruct ring metadata from compact ordering.
        self._n_valid = int(n_valid)
        if self._n_valid == 0:
            self._write_cursor = 0
        else:
            newest_bi = int(self._n_valid - 1)
            newest_slot = int(np.where((valid) & (buffer_indexes == newest_bi))[0][0])
            self._write_cursor = (newest_slot + 1) % self.max_size

        return buffer_indexes[slots]
    
    def _update_buffer_index(self, slots: int | np.ndarray | list[int]) -> np.ndarray:
        """
        Incremental buffer_index update for ring-ordered writes.
        Falls back to robust full rebuild if slots are not ring-contiguous.
        """
        slots = np.asarray(slots, dtype=np.int64).reshape(-1)
        if slots.size == 0:
            return np.empty(0, dtype=np.int64)

        if not self._is_ring_step_sequence(slots):
            return self._update_buffer_index_fallback(slots)

        buffer_indexes = self._buffer_index_cache
        valid = self._valid_cache

        n_new = int(slots.size)
        prev_n = int(self._n_valid)
        cap = self.max_size

        if n_new + prev_n > cap:
            # Wraping case: new slots will overwrite some of the oldest valid slots.
            buffer_indexes[:] = np.remainder(buffer_indexes - n_new, cap)

        else:
            # Non-wrapping case: just mark new slots as valid with correct buffer_index.
            buffer_indexes[slots] = np.arange(prev_n, prev_n + n_new, dtype=np.int64)
        
        valid[slots] = True

        self._n_valid = min(cap, prev_n + n_new)
        self._write_cursor = (self._write_cursor + n_new) % cap

        return buffer_indexes[slots]


    def _resolve_slots_to_write(self, n_samples: int) -> np.ndarray:
        if n_samples <= 0:
            raise ValueError("n_samples must be positive")

        if n_samples > self.max_size:
            raise ValueError(f"n_samples {n_samples} exceeds container size {self.max_size}")

        return (self._write_cursor + np.arange(n_samples, dtype=np.int64)) % self.max_size

    def _write_slot(self, slots: np.ndarray, values: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
        self._feature_cache[slots] = values
        self._timestamp_cache[slots] = timestamps
        self._valid_cache[slots] = True

        self._update_buffer_index(slots)
        return slots

    def push(
        self,
        sample: Container | np.ndarray,
        src_labels: list[str] | None = None,
        src_dtype: np.dtype | None = None,
        timestamps: np.ndarray | float | None = None,
    ) -> int | np.ndarray:
        if isinstance(sample, Container):
            dst_labels = tuple(str(x) for x in self.feature_labels)
            src_data = sample.data
            src_valid = sample._valid_cache
            if not np.any(src_valid):
                return np.empty(0, dtype=np.int64)

            src_slots = np.flatnonzero(src_valid)
            if src_slots.size > 1:
                src_bi = sample._buffer_index_cache
                src_slots = src_slots[np.argsort(src_bi[src_slots], kind="stable")]

            values = src_data.values[src_slots]
            ts = sample._timestamp_cache[src_slots]
            src_labels = tuple(str(x) for x in sample.feature_labels)

            if src_labels != dst_labels:
                take_idx = self._get_feature_take_indices(src_labels, dst_labels)
                values = np.take(values, take_idx, axis=1)

            values = np.asarray(values, dtype=self.data.dtype)

        else:
            n_features = len(self.feature_labels)

            if src_dtype is None:
                src_dtype = sample.dtype

            if src_dtype != self.data.dtype and not np.can_cast(src_dtype, self.data.dtype, casting="same_kind"):
                raise ValueError("Cannot cast src_dtype to self.data.dtype")

            values = np.asarray(sample, dtype=self.data.dtype)

            if values.ndim == 1:
                values = values.reshape(1, -1)
            elif values.ndim != 2:
                raise ValueError(f"sample ndarray must be 1D or 2D, got {values.shape}")

            if src_labels is not None:
                src_labels = tuple(str(x) for x in src_labels)
                dst_labels = tuple(str(x) for x in self.feature_labels)
                if src_labels != dst_labels:
                    take_idx = self._get_feature_take_indices(src_labels, dst_labels)
                    values = np.take(values, take_idx, axis=1)
            elif values.shape[1] != n_features:
                    raise ValueError(f"sample must have {n_features} features, got {values.shape[1]}")

            if timestamps is None:
                raise ValueError("timestamps ndarray is required when sample is ndarray")
            ts = np.asarray(timestamps, dtype=np.float64).reshape(-1)

        n_rows = values.shape[0]
        if n_rows == 0:
            return np.empty(0, dtype=np.int64)

        if ts.size != n_rows:
            raise ValueError(f"timestamps length must be {n_rows}, got {ts.size}")

        if n_rows > self.max_size:
            values = values[-self.max_size:]
            ts = ts[-self.max_size:]
            n_rows = self.max_size

        slots = self._resolve_slots_to_write(n_rows)
        written_slots = self._write_slot(slots, values, ts)

        return int(written_slots[0]) if written_slots.size == 1 else written_slots
    
    def push_from_msg(self, msg: ContainerMsg):
        values, ts, feature_labels, dtype, attrs = self.decode_container_msg_payload(msg)
        return self.push(values, src_labels=feature_labels, src_dtype=dtype, timestamps=ts)

    def clear(self) -> None:
        self._valid_cache[:] = False
        self._buffer_index_cache[:] = -1
        self._timestamp_cache[:] = 0.0

        self._write_cursor = 0
        self._n_valid = 0

    def read(self, index: int | slice | list[int] | np.ndarray | None = None, ordered: bool = True) -> xr.DataArray:
        if index is None:
            if ordered:
                slots = self._ordered_all_valid_slots()
            else:
                slots = np.flatnonzero(self._valid_cache)
            return self.data.isel(sample=slots)

        ordered_slots = self._ordered_all_valid_slots()
        n = ordered_slots.size

        if isinstance(index, (int, np.integer)):
            if index >= n or index < -n:
                raise IndexError(f"buffer_index {index} out of range [{-n}, {n - 1}]")
            slots = np.array([ordered_slots[index]], dtype=np.int64)
        elif isinstance(index, slice):
            slots = ordered_slots[index]
        else:
            buffer_indexes = np.asarray(index, dtype=np.int64)
            if buffer_indexes.ndim != 1:
                raise ValueError("index array must be 1-dimensional")
            if np.any((buffer_indexes < -n) | (buffer_indexes >= n)):
                raise IndexError(f"buffer_index out of range [{-n}, {n - 1}]")
            slots = ordered_slots[buffer_indexes]

        out = self.data.isel(sample=slots)
        if not ordered or out.sizes.get("sample", 0) <= 1:
            return out

        return out.isel(sample=np.argsort(out.coords["buffer_index"].values, kind="stable"))

    def read_slots(self, slots: int | slice | list[int] | np.ndarray) -> np.ndarray:
        return self._feature_cache[slots]
    
    def _read_ordered_numpy(self):
        n = int(self._n_valid)
        if n == 0:
            return self._feature_cache[:0], self._timestamp_cache[:0]

        start = (self._write_cursor - n) % self.max_size
        end = start + n

        if end <= self.max_size:
            # zero-copy views
            return self._feature_cache[start:end], self._timestamp_cache[start:end]

        # one allocation each
        v = np.concatenate((self._feature_cache[start:], self._feature_cache[:end % self.max_size]), axis=0)
        t = np.concatenate((self._timestamp_cache[start:], self._timestamp_cache[:end % self.max_size]), axis=0)
        return v, t
    
    def to_msg(self) -> ContainerMsg:

        values, ts = self._read_ordered_numpy()

        dtype = values.dtype
        if dtype not in DTYPE_TO_CODE:
            raise ValueError(f"Unsupported dtype {dtype}")

        self.msg.name = self.name or ""
        self.msg.container_type = self.container_type

        self.msg.feature_labels = self.feature_labels
        self.msg.max_size = int(self.max_size)
        self.msg.n_rows = int(values.shape[0])

        self.msg.dtype_code = DTYPE_TO_CODE[dtype]
        self.msg.little_endian = (values.dtype.byteorder in ("<", "=") and np.little_endian)

        payload = array('B')  # Clear previous contents
        payload.frombytes(values.tobytes(order="C"))
        self.msg.data_bytes = payload

        ts_payload = array('d')
        ts_payload.frombytes(ts.tobytes(order="C"))
        self.msg.timestamps = ts_payload

        attrs = self.data.attrs
        if not attrs:
            if self._attrs_written:
                self.msg.attrs_keys = []
                self.msg.attrs_values = []
                self._attrs_written = False
                self._attrs_pairs_cache = None
        else:
            pairs = tuple((str(k), str(v)) for k, v in attrs.items())
            if pairs != self._attrs_pairs_cache:
                self.msg.attrs_keys = [k for k, _ in pairs]
                self.msg.attrs_values = [v for _, v in pairs]
                self._attrs_pairs_cache = pairs
                self._attrs_written = True
        return self.msg

    @staticmethod
    def decode_container_msg_payload(msg):
        dtype = CODE_TO_DTYPE[msg.dtype_code]
        n_rows = int(msg.n_rows)
        n_features = len(msg.feature_labels)

        # Minimal-copy path from uint8[] payload
        raw = np.asarray(msg.data_bytes, dtype=np.uint8)
        values = np.frombuffer(raw.tobytes(), dtype=dtype)
        if values.size != n_rows * n_features:
            raise ValueError("payload size mismatch")
        values = values.reshape(n_rows, n_features)

        ts = np.asarray(msg.timestamps, dtype=np.float64)
        if ts.size != n_rows:
            raise ValueError("timestamps size mismatch")

        # Endianness guard
        if bool(msg.little_endian) != bool(np.little_endian):
            values = values.byteswap().newbyteorder()

        if msg.attrs_keys and msg.attrs_values:
            attrs = {k: v for k, v in zip(msg.attrs_keys, msg.attrs_values)}
        else:
            attrs = {}

        return values, ts, list(msg.feature_labels), dtype, attrs
    
    @classmethod
    def from_msg(cls, msg: ContainerMsg, max_size: None | int = None) -> Container | None:

        # Return None if message is empty (no data, timestamps, or feature labels)
        if len(msg.data_bytes) == 0 or len(msg.timestamps) == 0 or len(msg.feature_labels) == 0:
            return None

        values, ts, feature_labels, dtype, attrs = cls.decode_container_msg_payload(msg)

        container = cls(
            name=msg.name,
            max_size=max_size if max_size is not None else msg.max_size,
            container_type=msg.container_type,
            data_type=dtype,
            labels=feature_labels,
            attrs=attrs
        )
        container.push(values, src_labels=feature_labels, src_dtype=dtype, timestamps=ts)
        return container
    
    @classmethod
    def from_dataarray(cls, data_array: xr.DataArray, container_type:str = "", name: str = "") -> Container:
        if "sample" not in data_array.dims or "features" not in data_array.dims:
            raise ValueError("data_array must have the 'sample' and 'features' dimensions")

        feature_labels = data_array.coords["features"].values.tolist()
        container = cls(
            name=name if name else data_array.name,
            max_size=data_array.sizes["sample"],
            container_type=container_type,
            data_type=data_array.dtype,
            labels=feature_labels,
            attrs=data_array.attrs
        )
        container.push(data_array.values, src_labels=feature_labels, src_dtype=data_array.dtype, timestamps=data_array.coords.get("timestamp", np.zeros(data_array.sizes["sample"])))
        return container
    
    def __repr__(self) -> str:
        labels = [str(x) for x in self.feature_labels]
        header = (
            f"Container(name={self.name!r}, type={self.container_type!r}, "
            f"dtype={self.data.dtype}, size={self.size}/{self.max_size}, "
            f"n_features={len(labels)})"
        )

        ordered_slots = self._ordered_all_valid_slots()
        if ordered_slots.size == 0:
            return f"{header}\n<empty>"

        max_rows = 10
        shown_slots = ordered_slots[:max_rows]

        columns = ["buffer_index", "slot", "timestamp", *labels]
        rows: list[list[str]] = []

        values = self._feature_cache[shown_slots]
        ts = self._timestamp_cache[shown_slots]
        bi = self._buffer_index_cache[shown_slots]

        for i, slot in enumerate(shown_slots):
            row = [
                str(int(bi[i])),
                str(int(slot)),
                f"{float(ts[i]):.6g}",
            ]
            for v in values[i]:
                if isinstance(v, (np.floating, float)):
                    row.append(f"{float(v):.6g}")
                else:
                    row.append(str(v))
            rows.append(row)

        table = _format_debug_table(columns, rows)
        suffix = ""
        if ordered_slots.size > max_rows:
            suffix = f"\n... {ordered_slots.size - max_rows} more row(s) not shown"

        return f"{header}\n{table}{suffix}"

## Helper methods for Containers

def consolidate_containers(containers: list[Container], write_container: Container | None = None, labels_mode: str = "original", name: str | None = None, container_type: str | None = None, attrs: dict = {}) -> Container:
    """Consolidate multiple containers into a single container by concatenating their features and aligning their samples.

    Labels mode determines how to handle feature labels when write_container is provided:
        - "extend": Union of labels from write_container and all containers. New labels are added to write_container if not already present
        - "original": Use labels from write_container as-is. Only features matching these labels will be included in the output. 
        Raises error if any of the write container labels are missing from the input containers.

    """
    new_labels = []
    data = [container._read_ordered_numpy() for container in containers]
    features = np.concatenate([features for features, timestamps in data], axis=1) # Concatenate features along feature dimension
    timestamps_list = [timestamps.reshape(-1, 1) for features, timestamps in data]
    timestamps = np.concatenate(timestamps_list, axis=1) # Concatenate timestamps for each sample across containers (shape: n_samples x n_containers)
    oldest_timestamps = np.min(timestamps, axis=1) if timestamps.size > 0 else 0.0 # Get the oldest timestamp for each sample across all containers
    for container in containers:
        new_labels.extend([f"{container.name}:{label}" for label in container.feature_labels])



    if write_container:
        name = write_container.name
        labels = write_container.feature_labels
        dtype = write_container.data.dtype
        attrs = {**write_container.data.attrs, **attrs}
        container_type = write_container.container_type

        labels_present = set(labels).issubset(set(new_labels))

        if not labels_present:
            raise ValueError("Consolidation with incomplete labels is not supported yet. Please ensure all labels in the write_container are present in the input containers.")
        
        if labels_mode == "extend":
            pass # A new container will be created with the extended set of labels, so no need to modify the write_container's labels.  
        elif labels_mode == "original":
            write_container.push(features, src_labels=new_labels, src_dtype=dtype, timestamps=oldest_timestamps)
            return write_container
        else:
            raise ValueError(f"Invalid labels_mode: {labels_mode}")
    
    new_container = Container(
        name=name or "consolidated",
        max_size=features.shape[0],
        container_type=container_type or "consolidated",
        data_type=features.dtype,
        labels=new_labels,
        attrs=attrs,
    )
    new_container.push(features, src_labels=new_labels, src_dtype=features.dtype, timestamps=oldest_timestamps)
    return new_container

    











class MultiContainer:
    """A 2D FIFO container for traces of samples.
    
    Shape: (max_traces, max_size, n_features)
    - Each trace contains up to max_size samples
    - Each container can hold up to max_traces traces
    - When full, oldest traces are overwritten in FIFO order
    """
    
    @property
    def name(self) -> str | None:
        return self.data.name

    @name.setter
    def name(self, value: str | None) -> None:
        self.data.name = value

    @property
    def feature_labels(self) -> list[str]:
        return self._feature_labels_cache.tolist()
    
    @property
    def n_traces(self) -> int:
        """Number of valid traces in the container."""
        return int(self._n_valid_traces)
    
    @property
    def trace_sizes(self) -> np.ndarray:
        """Array of sizes for each trace (number of valid samples per trace)."""
        return self._n_valid_samples.copy()

    def __init__(
        self,
        name: str,
        max_traces: int,
        max_size: int,
        container_type: str,
        data_type=np.float64,
        labels: list = None,
        attrs: dict = None,
    ) -> None:
        self.feature_dim = "features"
        self.container_type = container_type
        feature_labels = labels if labels is not None else []
        shape = (max_traces, max_size, len(feature_labels))

        coords = {
            self.feature_dim: feature_labels,
            "timestamp": (("trace", "sample"), np.zeros((max_traces, max_size), dtype=np.float64)),
            "buffer_index_trace": ("trace", np.full(max_traces, -1, dtype=np.uint32)),
            "trace_valid": ("trace", np.zeros(max_traces, dtype=bool)),
            "sample_valid": (("trace", "sample"), np.zeros((max_traces, max_size), dtype=bool)),
        }
        
        data = np.full(shape, np.nan, dtype=data_type)

        self.data = xr.DataArray(
            data=data,
            dims=["trace", "sample", self.feature_dim],
            coords=coords,
            name=name,
            attrs=attrs if attrs is not None else {},
        )

        self._feature_index_cache: dict[tuple[tuple[str, ...], tuple[str, ...]], np.ndarray] = {}
        self._attrs_pairs_cache = None
        self._attrs_written = False

        # Ring buffer metadata for traces
        self._write_cursor_trace = 0
        self._n_valid_traces = 0

        # Track valid samples per trace (size of each trace)
        self._n_valid_samples = np.zeros(max_traces, dtype=np.int64)

        self._refresh_cache()

    def _refresh_cache(self) -> None:
        self._feature_cache = self.data.values
        self._timestamp_cache = self.data.coords["timestamp"].values
        self._trace_valid_cache = self.data.coords["trace_valid"].values
        self._sample_valid_cache = self.data.coords["sample_valid"].values
        self._buffer_index_trace_cache = self.data.coords["buffer_index_trace"].values
        self._feature_labels_cache = self.data.coords[self.feature_dim].values
        self.max_traces = int(self.data.sizes["trace"])
        self.max_size = int(self.data.sizes["sample"])

    def _get_feature_take_indices(
        self,
        src_labels: tuple[str, ...],
        dst_labels: tuple[str, ...],
    ) -> np.ndarray:
        """Reuse Container's feature indexing logic."""
        cache_key = (src_labels, dst_labels)
        cached = self._feature_index_cache.get(cache_key)
        if cached is not None:
            return cached

        src_pos = {label: i for i, label in enumerate(src_labels)}
        try:
            take_idx = np.fromiter(
                (src_pos[label] for label in dst_labels),
                dtype=np.int64,
                count=len(dst_labels),
            )
        except KeyError as exc:
            missing = str(exc.args[0])
            raise ValueError(f"Missing destination feature in source container: {missing}") from exc

        self._feature_index_cache[cache_key] = take_idx
        return take_idx

    def push(
        self,
        trace_data: Container | np.ndarray,
        src_labels: list[str] | None = None,
        src_dtype: np.dtype | None = None,
        timestamps: np.ndarray | None = None,
    ) -> int:
        """Push a trace (Container or ndarray) into the MultiContainer.
        
        Args:
            trace_data: A Container or 2D ndarray representing one trace
            src_labels: Feature labels for the source data (if ndarray)
            src_dtype: Data type of source (if ndarray)
            timestamps: Timestamps for each sample in the trace
        
        Returns:
            The trace buffer_index where the trace was written, or -1 if no data.
        """
        # Extract values and timestamps from source
        if isinstance(trace_data, Container):
            dst_labels = tuple(str(x) for x in self.feature_labels)
            src_data = trace_data.data
            src_valid = trace_data._valid_cache
            
            if not np.any(src_valid):
                return -1

            src_slots = np.flatnonzero(src_valid)
            if src_slots.size > 1:
                src_bi = trace_data._buffer_index_cache
                src_slots = src_slots[np.argsort(src_bi[src_slots], kind="stable")]

            values = src_data.values[src_slots]
            ts = trace_data._timestamp_cache[src_slots]
            src_labels = tuple(str(x) for x in trace_data.feature_labels)

            if src_labels != dst_labels:
                take_idx = self._get_feature_take_indices(src_labels, dst_labels)
                values = np.take(values, take_idx, axis=1)

            values = np.asarray(values, dtype=self.data.dtype)
            n_samples = values.shape[0]

        else:
            # Handle ndarray input
            n_features = len(self.feature_labels)

            if src_dtype is None:
                src_dtype = trace_data.dtype

            if src_dtype != self.data.dtype and not np.can_cast(src_dtype, self.data.dtype, casting="same_kind"):
                raise ValueError("Cannot cast src_dtype to self.data.dtype")

            values = np.asarray(trace_data, dtype=self.data.dtype)

            if values.ndim == 1:
                values = values.reshape(1, -1)
            elif values.ndim != 2:
                raise ValueError(f"trace_data ndarray must be 1D or 2D, got {values.shape}")

            if src_labels is not None:
                src_labels = tuple(str(x) for x in src_labels)
                dst_labels = tuple(str(x) for x in self.feature_labels)
                if src_labels != dst_labels:
                    take_idx = self._get_feature_take_indices(src_labels, dst_labels)
                    values = np.take(values, take_idx, axis=1)
            elif values.shape[1] != n_features:
                raise ValueError(f"trace_data must have {n_features} features, got {values.shape[1]}")

            if timestamps is None:
                raise ValueError("timestamps ndarray is required when trace_data is ndarray")
            
            ts = np.asarray(timestamps, dtype=np.float64).reshape(-1)
            n_samples = values.shape[0]

        if n_samples == 0:
            return -1

        if ts.size != n_samples:
            raise ValueError(f"timestamps length must be {n_samples}, got {ts.size}")

        if n_samples > self.max_size:
            values = values[-self.max_size:]
            ts = ts[-self.max_size:]
            n_samples = self.max_size

        # Resolve trace slot and update ring buffer metadata
        trace_slot = self._write_cursor_trace
        prev_n_traces = int(self._n_valid_traces)
        
        # If this trace was already valid, we're overwriting it
        # Otherwise, we're adding a new trace
        is_new_trace = not bool(self._trace_valid_cache[trace_slot])

        # Clear the trace slot before writing (invalidate old samples)
        self._sample_valid_cache[trace_slot, :] = False

        # Write data to trace
        sample_slots = np.arange(n_samples, dtype=np.int64)
        self._feature_cache[trace_slot, sample_slots] = values
        self._timestamp_cache[trace_slot, sample_slots] = ts
        self._sample_valid_cache[trace_slot, sample_slots] = True

        # Update trace-level metadata
        self._trace_valid_cache[trace_slot] = True
        self._n_valid_samples[trace_slot] = n_samples

        # Update trace buffer index
        if is_new_trace:
            self._buffer_index_trace_cache[trace_slot] = prev_n_traces
            self._n_valid_traces = min(self.max_traces, prev_n_traces + 1)

        trace_buffer_idx = int(self._buffer_index_trace_cache[trace_slot])

        # Advance trace cursor
        self._write_cursor_trace = (self._write_cursor_trace + 1) % self.max_traces

        return trace_buffer_idx

    def clear(self) -> None:
        """Clear all traces."""
        self._trace_valid_cache[:] = False
        self._sample_valid_cache[:] = False
        self._buffer_index_trace_cache[:] = -1
        self._timestamp_cache[:] = 0.0
        self._write_cursor_trace = 0
        self._n_valid_traces = 0
        self._n_valid_samples[:] = 0

    def _flatten_traces(self, trace_slots: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Flatten data from specified trace slots.
        
        Args:
            trace_slots: Array of trace indices to flatten
        
        Returns:
            Tuple of (flattened_values, flattened_timestamps)
        """
        all_values = []
        all_timestamps = []
        
        for trace_slot in trace_slots:
            trace_slot = int(trace_slot)
            valid_sample_indices = np.flatnonzero(self._sample_valid_cache[trace_slot])
            
            if valid_sample_indices.size == 0:
                continue
            
            all_values.append(self._feature_cache[trace_slot, valid_sample_indices])
            all_timestamps.append(self._timestamp_cache[trace_slot, valid_sample_indices])
        
        if all_values:
            concat_values = np.concatenate(all_values, axis=0)
            concat_ts = np.concatenate(all_timestamps, axis=0)
        else:
            concat_values = np.empty((0, len(self.feature_labels)), dtype=self.data.dtype)
            concat_ts = np.empty(0, dtype=np.float64)
        
        return concat_values, concat_ts

    def read_flattened(self, trace_index: int | slice | list[int] | np.ndarray | None = None) -> xr.DataArray:
        """Read traces flattened into a single DataArray.
        
        Args:
            trace_index: Trace selection. Can be:
                - None: include all valid traces
                - int: single trace by buffer_index
                - slice: range of traces by buffer_index
                - array: specific traces by buffer_index
        
        Returns:
            xr.DataArray with dims (sample, features) containing flattened trace data.
        """
        ordered_trace_slots = self._ordered_all_valid_trace_slots()
        n = ordered_trace_slots.size
        
        if trace_index is None:
            trace_slots = ordered_trace_slots
        elif isinstance(trace_index, int):
            if trace_index >= n or trace_index < -n:
                raise IndexError(f"trace buffer_index {trace_index} out of range [{-n}, {n - 1}]")
            trace_slots = np.array([ordered_trace_slots[trace_index]], dtype=np.int64)
        elif isinstance(trace_index, slice):
            trace_slots = ordered_trace_slots[trace_index]
        else:
            buffer_indexes = np.asarray(trace_index, dtype=np.int64)
            if buffer_indexes.ndim != 1:
                raise ValueError("trace_index array must be 1-dimensional")
            if np.any((buffer_indexes < -n) | (buffer_indexes >= n)):
                raise IndexError(f"trace buffer_index out of range [{-n}, {n - 1}]")
            trace_slots = ordered_trace_slots[buffer_indexes]
        
        concat_values, concat_ts = self._flatten_traces(trace_slots)
        
        coords = {
            self.feature_dim: self.feature_labels,
            "timestamp": ("sample", concat_ts),
        }
        
        result = xr.DataArray(
            data=concat_values,
            dims=["sample", self.feature_dim],
            coords=coords,
            name=self.name,
            attrs=self.data.attrs,
        )
        
        return result

    def _ordered_all_valid_trace_slots(self) -> np.ndarray:
        """Get all valid trace slots in chronological order (oldest to newest)."""
        n = int(self._n_valid_traces)
        if n == 0:
            return np.empty(0, dtype=np.int64)

        start = (self._write_cursor_trace - n) % self.max_traces
        end = start + n

        if end <= self.max_traces:
            return np.arange(start, end, dtype=np.int64)

        first = np.arange(start, self.max_traces, dtype=np.int64)
        second = np.arange(0, end % self.max_traces, dtype=np.int64)
        return np.concatenate((first, second))

    def __repr__(self) -> str:
        labels = [str(x) for x in self.feature_labels]
        header = (
            f"MultiContainer(name={self.name!r}, type={self.container_type!r}, "
            f"dtype={self.data.dtype}, traces={self.n_traces}/{self.max_traces}, "
            f"max_size={self.max_size}, n_features={len(labels)})"
        )

        ordered_traces = self._ordered_all_valid_trace_slots()
        if ordered_traces.size == 0:
            return f"{header}\n<empty>"

        max_rows = 12
        shown = ordered_traces[:max_rows]

        columns = ["trace_buffer_index", "trace_slot", "n_samples", "t_first", "t_last"]
        rows: list[list[str]] = []

        for trace_slot in shown:
            trace_slot = int(trace_slot)
            n_samples = int(self._n_valid_samples[trace_slot])
            trace_bi = int(self._buffer_index_trace_cache[trace_slot])

            if n_samples > 0:
                ts = self._timestamp_cache[trace_slot, :n_samples]
                t_first = f"{float(ts[0]):.6g}"
                t_last = f"{float(ts[-1]):.6g}"
            else:
                t_first = "-"
                t_last = "-"

            rows.append([
                str(trace_bi),
                str(trace_slot),
                str(n_samples),
                t_first,
                t_last,
            ])

        table = _format_debug_table(columns, rows)
        suffix = ""
        if ordered_traces.size > max_rows:
            suffix = f"\n... {ordered_traces.size - max_rows} more trace(s) not shown"

        return f"{header}\n{table}{suffix}"
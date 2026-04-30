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
        timestamps: np.ndarray | None = None,
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

        if isinstance(index, int):
            if index < 0 or index >= ordered_slots.size:
                raise IndexError(f"buffer_index {index} out of range [0, {ordered_slots.size - 1}]")
            slots = np.array([ordered_slots[index]], dtype=np.int64)
        elif isinstance(index, slice):
            slots = ordered_slots[index]
        else:
            buffer_indexes = np.asarray(index, dtype=np.int64)
            if buffer_indexes.ndim != 1:
                raise ValueError("index array must be 1-dimensional")
            if np.any((buffer_indexes < 0) | (buffer_indexes >= ordered_slots.size)):
                raise IndexError(f"buffer_index out of range [0, {ordered_slots.size - 1}]")
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
        self.msg.container_type = str(self.data.attrs.get("type", ""))

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
    def from_msg(cls, msg: ContainerMsg, max_size: None | int = None) -> Container:
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
string name
string container_type

# Feature axis
string[] feature_labels

# Ring capacity
uint32 max_size

# Number of serialized rows (valid rows only, ordered oldest->newest)
uint32 n_rows

# Data type code for payload in "data_bytes"
# 1=float32, 2=float64, 3=int32, 4=int64, 5=uint8, 6=bool
uint8 dtype_code
bool little_endian

# Flattened row-major bytes for shape [n_rows, len(feature_labels)]
uint8[] data_bytes

# Per-row metadata aligned with n_rows
float64[] timestamps

# Optional generic metadata
string[] attrs_keys
string[] attrs_values
"""Model-agnostic P-Node sensitivity and response-surface calculations."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd

from core.container import Container


_PNODE_SUCCESS_COLUMNS = ["pnode", "iteration", "success"]
_PNODE_CUMULATIVE_COLUMNS = [
    "pnode",
    "iteration",
    "cumulative_points",
    "cumulative_antipoints",
    "net_points",
]


def _load_pnode_success(
    source: str | Path | pd.DataFrame,
) -> pd.DataFrame:
    """Load and normalize a P-Node success table."""
    if isinstance(source, pd.DataFrame):
        table = source.copy()
    else:
        path = Path(source).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"P-Node success file does not exist: {path}")
        table = pd.read_csv(path, sep="\t", comment="#")

    if set(_PNODE_SUCCESS_COLUMNS).issubset(table.columns):
        result = table[_PNODE_SUCCESS_COLUMNS].copy()
    else:
        required = {"Iteration", "Ident", "Success"}
        missing = required - set(table.columns)
        if missing:
            raise ValueError(
                f"P-Node success table is missing columns: {sorted(missing)}"
            )
        result = table.rename(
            columns={"Ident": "pnode", "Iteration": "iteration", "Success": "success"}
        )[_PNODE_SUCCESS_COLUMNS].copy()

    def parse_success(value: object) -> bool:
        if isinstance(value, (bool, np.bool_)):
            return bool(value)
        normalized = str(value).strip().lower()
        if normalized == "true":
            return True
        if normalized == "false":
            return False
        raise ValueError(f"Invalid P-Node success value: {value!r}")

    result["success"] = result["success"].map(parse_success)
    return result


def pnode_success_table(
    source: str | Path | pd.DataFrame,
) -> pd.DataFrame:
    """Return one normalized success event row for every P-Node occurrence."""
    return _load_pnode_success(source)


def cumulative_pnode_success_table(
    source: str | Path | pd.DataFrame,
) -> pd.DataFrame:
    """Return cumulative success, failure, and net points for each P-Node.

    Each P-Node is represented at every iteration from its first observation
    through the final iteration in the input. Values are carried forward on
    iterations without an event, so plotting the table preserves flat
    sections.
    """
    events = _load_pnode_success(source)
    if events.empty:
        return pd.DataFrame(columns=_PNODE_CUMULATIVE_COLUMNS)

    events["iteration"] = pd.to_numeric(events["iteration"], errors="raise")
    if not (events["iteration"] % 1 == 0).all():
        raise ValueError("P-Node iterations must be whole numbers")
    events["iteration"] = events["iteration"].astype(int)
    events = events.assign(
        points=events["success"].astype(int),
        antipoints=(~events["success"]).astype(int),
    )
    event_totals = (
        events.groupby(["pnode", "iteration"], sort=False)[["points", "antipoints"]]
        .sum()
        .reset_index()
    )
    final_iteration = int(events["iteration"].max())
    expanded = pd.concat(
        [
            pd.DataFrame({
                "pnode": pnode,
                "iteration": range(
                    int(pnode_events["iteration"].min()),
                    final_iteration + 1,
                ),
            })
            for pnode, pnode_events in event_totals.groupby("pnode", sort=False)
        ],
        ignore_index=True,
    )
    result = expanded.merge(
        event_totals,
        on=["pnode", "iteration"],
        how="left",
    ).fillna({"points": 0, "antipoints": 0})
    result["cumulative_points"] = result.groupby("pnode")["points"].cumsum()
    result["cumulative_antipoints"] = (
        result.groupby("pnode")["antipoints"].cumsum()
    )
    result["net_points"] = (
        result["cumulative_points"] - result["cumulative_antipoints"]
    )
    return result[_PNODE_CUMULATIVE_COLUMNS]


def _as_frame(X, feature_labels: Iterable[str] | None = None) -> pd.DataFrame:
    if isinstance(X, pd.DataFrame):
        frame = X.copy()
    else:
        array = np.asarray(X, dtype=float)
        if array.ndim != 2:
            raise ValueError("X must be a 2D array or pandas DataFrame")
        if feature_labels is None:
            feature_labels = [f"feature_{i}" for i in range(array.shape[1])]
        frame = pd.DataFrame(array, columns=list(feature_labels))
    if frame.empty:
        raise ValueError("X must contain at least one row")
    if not np.all(frame.apply(lambda column: pd.api.types.is_numeric_dtype(column))):
        raise TypeError("All analysis features must be numeric")
    return frame


def evaluate_space(space, X, feature_labels: Iterable[str] | None = None) -> np.ndarray:
    """Evaluate a reconstructed Space on rows of numeric feature data."""
    frame = _as_frame(X, feature_labels)
    labels = list(frame.columns)
    data = Container(
        name=f"{getattr(space, 'ident', 'space')}_analysis",
        max_size=len(frame),
        container_type="perception",
        labels=labels,
    )
    data.push(
        frame.to_numpy(dtype=float),
        src_labels=labels,
        timestamps=np.arange(len(frame), dtype=float),
    )
    predictions = np.asarray(space.get_probability(data)).reshape(-1)
    if predictions.size != len(frame):
        raise ValueError(
            f"Space returned {predictions.size} predictions for {len(frame)} rows"
        )
    return predictions


def permutation_sensitivity(
    space,
    X,
    *,
    feature_labels: Iterable[str] | None = None,
    n_repeats: int = 5,
    random_state: int | None = 0,
) -> pd.DataFrame:
    """Estimate feature sensitivity by permuting one feature at a time."""
    if n_repeats < 1:
        raise ValueError("n_repeats must be positive")
    frame = _as_frame(X, feature_labels)
    baseline = evaluate_space(space, frame)
    rng = np.random.default_rng(random_state)
    rows = []
    for feature in frame.columns:
        changes = []
        for _ in range(n_repeats):
            perturbed = frame.copy()
            perturbed[feature] = rng.permutation(perturbed[feature].to_numpy())
            changes.append(np.abs(evaluate_space(space, perturbed) - baseline).mean())
        rows.append({
            "feature": feature,
            "mean_absolute_change": float(np.mean(changes)),
            "std_absolute_change": float(np.std(changes)),
        })
    return pd.DataFrame(rows).sort_values(
        "mean_absolute_change", ascending=False, ignore_index=True
    )


def response_surface(
    space,
    X,
    features: tuple[str, str] | list[str],
    *,
    feature_labels: Iterable[str] | None = None,
    grid_points: int = 40,
    quantiles: tuple[float, float] = (0.02, 0.98),
    baseline: str | pd.Series | np.ndarray = "median",
) -> dict[str, np.ndarray | str]:
    """Evaluate a two-feature conditional response surface."""
    if len(features) != 2 or features[0] == features[1]:
        raise ValueError("features must contain two distinct feature names")
    if grid_points < 2:
        raise ValueError("grid_points must be at least 2")
    frame = _as_frame(X, feature_labels)
    first, second = features
    missing = {first, second} - set(frame.columns)
    if missing:
        raise KeyError(f"Unknown response-surface features: {sorted(missing)}")
    if not 0 <= quantiles[0] < quantiles[1] <= 1:
        raise ValueError("quantiles must satisfy 0 <= low < high <= 1")

    if isinstance(baseline, str):
        if baseline != "median":
            raise ValueError(
                "baseline must be 'median', a Series, or a numeric array"
            )
        base = frame.median(numeric_only=True)
    elif isinstance(baseline, pd.Series):
        base = baseline.reindex(frame.columns)
    else:
        values = np.asarray(baseline, dtype=float)
        if values.shape != (len(frame.columns),):
            raise ValueError("baseline array must have one value per feature")
        base = pd.Series(values, index=frame.columns)
    if base.isna().any():
        raise ValueError("baseline must provide a finite value for every feature")

    x_values = np.linspace(*frame[first].quantile(list(quantiles)), grid_points)
    y_values = np.linspace(*frame[second].quantile(list(quantiles)), grid_points)
    grid = pd.DataFrame(
        np.tile(base.to_numpy(dtype=float), (grid_points * grid_points, 1)),
        columns=frame.columns,
    )
    xx, yy = np.meshgrid(x_values, y_values)
    grid[first] = xx.ravel()
    grid[second] = yy.ravel()
    predictions = evaluate_space(space, grid).reshape(grid_points, grid_points)
    return {
        "x": x_values,
        "y": y_values,
        "prediction": predictions,
        "x_feature": first,
        "y_feature": second,
    }

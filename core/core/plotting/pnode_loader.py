"""Reconstruct P-Node spaces from exported samples and model snapshots."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from core.container import Container
from cognitive_nodes.space import ANNSpace, ClosestPointBasedSpace, PointBasedSpace

from .result_loader import ExperimentResults, load_results


@dataclass
class LoadedPNode:
    """A reconstructed space and the samples used to populate it."""

    name: str
    iteration: float
    space: PointBasedSpace
    samples: pd.DataFrame
    model_file: Path | None = None

    @property
    def feature_labels(self) -> list[str]:
        return [
            column for column in self.samples.columns
            if column not in {"source_file", "Iteration", "Ident", "confidence"}
        ]


def _resolve_space_class(space_class: Any, model_file: Path | None):
    if space_class is None:
        return ANNSpace if model_file is not None else ClosestPointBasedSpace
    if isinstance(space_class, str):
        candidates = {
            "ANNSpace": ANNSpace,
            "ClosestPointBasedSpace": ClosestPointBasedSpace,
            "PointBasedSpace": PointBasedSpace,
        }
        try:
            return candidates[space_class]
        except KeyError as error:
            raise ValueError(f"Unsupported space class: {space_class}") from error
    return space_class


def _model_for_iteration(
    model_files: list[Path], pnode_name: str, iteration: float | None
) -> Path | None:
    candidates = [
        path for path in model_files
        if path.name.startswith(f"{pnode_name}_iter_")
    ]
    if not candidates:
        return None
    if iteration is not None:
        exact = [
            path for path in candidates
            if path.stem.endswith(f"_iter_{int(iteration)}")
        ]
        if exact:
            return exact[0]
    return max(
        candidates,
        key=lambda path: int(path.stem.rsplit("_iter_", 1)[1]),
    )


def load_pnode(
    results: ExperimentResults | str | Path,
    pnode_name: str,
    *,
    iteration: float | None = None,
    space_class: Any = None,
    model_file: str | Path | None = None,
    max_samples: int | None = None,
    device: str = "cpu",
) -> LoadedPNode:
    """Reconstruct one P-Node from exported content and an optional model."""
    bundle = results if isinstance(results, ExperimentResults) else load_results(results)
    content = bundle.table("pnodes_content")
    required = {"Ident", "Iteration", "confidence"}
    missing = required - set(content.columns)
    if missing:
        raise ValueError(f"P-Node table is missing columns: {sorted(missing)}")

    rows = content[content["Ident"] == pnode_name].copy()
    if rows.empty:
        available = sorted(content["Ident"].dropna().unique())
        raise KeyError(f"P-Node {pnode_name!r} not found. Available: {available}")

    selected_iteration = (
        float(rows["Iteration"].max()) if iteration is None else float(iteration)
    )
    rows = rows[rows["Iteration"] == selected_iteration].copy()
    if rows.empty:
        available = sorted(
            content.loc[content["Ident"] == pnode_name, "Iteration"].unique()
        )
        raise KeyError(
            f"Iteration {iteration} not found for {pnode_name!r}. "
            f"Available: {available}"
        )
    if max_samples is not None:
        if max_samples < 1:
            raise ValueError("max_samples must be positive")
        rows = rows.tail(max_samples).copy()

    features = [
        column for column in rows.columns
        if column not in {"source_file", "Iteration", "Ident", "confidence"}
    ]
    values = rows[features + ["confidence"]].to_numpy(dtype=float)
    data = Container(
        name=f"{pnode_name}_data",
        max_size=len(values),
        container_type="space",
        labels=features + ["confidence"],
    )
    data.push(
        values,
        src_labels=features + ["confidence"],
        timestamps=np.arange(len(values), dtype=float),
    )

    resolved_model = (
        Path(model_file).expanduser().resolve()
        if model_file is not None
        else _model_for_iteration(bundle.model_files, pnode_name, selected_iteration)
    )
    if resolved_model is not None and not resolved_model.is_file():
        raise FileNotFoundError(f"Model snapshot does not exist: {resolved_model}")

    cls = _resolve_space_class(space_class, resolved_model)
    kwargs = {"ident": pnode_name, "device": device} if cls is ANNSpace else {
        "ident": pnode_name
    }
    if cls is ANNSpace and resolved_model is None:
        raise ValueError("ANNSpace requires model_file or a matching model snapshot")
    space = (
        cls(model_file=str(resolved_model), **kwargs)
        if resolved_model
        else cls(**kwargs)
    )
    if space._data is None:
        space._data = data
    else:
        space._data.clear()
        space._data.push(
            values,
            src_labels=features + ["confidence"],
            timestamps=np.arange(len(values), dtype=float),
        )
    return LoadedPNode(
        name=pnode_name,
        iteration=selected_iteration,
        space=space,
        samples=rows,
        model_file=resolved_model,
    )

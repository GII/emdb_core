"""Loading utilities for EMDB experiment result directories.

The loader deliberately returns data rather than plotting it. This keeps the
same result bundle useful to notebooks, reports, and a future dashboard.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import pandas as pd


_TABULAR_FILES = {
    "trials": "trials_*.txt",
    "pnodes_confidence": "pnodes_confidence_*.txt",
    "subgoal_events": "subgoal_events_*.txt",
    "pnodes_content": "pnodes_content_*.txt",
    "goals_content": "goals_content_*.txt",
    "pnodes_success": "pnodes_success_*.txt",
    "neighbors": "neighbors_[0-9]*.txt",
    "neighbors_full": "neighbors_full_*.txt",
    "goodness": "goodness_*.txt",
    "dataset": "dataset_*.csv",
    "step_stats": "step_stats*.csv",
}


@dataclass
class ExperimentResults:
    """All discovered result files for one experiment execution."""

    root: Path
    data: dict[str, pd.DataFrame] = field(default_factory=dict)
    paths: dict[str, list[Path]] = field(default_factory=dict)
    model_files: list[Path] = field(default_factory=list)
    log_files: list[Path] = field(default_factory=list)
    yaml_files: list[Path] = field(default_factory=list)

    def table(self, name: str) -> pd.DataFrame:
        """Return a loaded table, raising a useful error when it is absent."""
        if name not in self.data:
            available = ", ".join(sorted(self.data)) or "none"
            raise KeyError(f"No table named {name!r}. Available tables: {available}")
        return self.data[name]


@dataclass
class MultiRunResults:
    """Independently loaded executions discovered below a collection root."""

    root: Path
    runs: dict[str, ExperimentResults] = field(default_factory=dict)

    def run(self, run_id: str) -> ExperimentResults:
        """Return one execution, raising a useful error when it is absent."""
        try:
            return self.runs[run_id]
        except KeyError as error:
            available = ", ".join(sorted(self.runs)) or "none"
            raise KeyError(
                f"No run named {run_id!r}. Available runs: {available}"
            ) from error


def _read_table(paths: list[Path], name: str) -> pd.DataFrame:
    frames = []
    for path in paths:
        if path.stat().st_size == 0:
            continue
        if name in {"dataset", "step_stats"}:
            frame = pd.read_csv(path)
        else:
            frame = pd.read_csv(path, sep=None, engine="python", comment="#")
        frame = frame.loc[:, ~frame.columns.astype(str).str.match(r"^Unnamed")]
        frame.insert(0, "source_file", str(path))
        frames.append(frame)
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def load_results(root: str | Path, *, recursive: bool = True) -> ExperimentResults:
    """Discover and load standard EMDB result files below ``root``.

    ``root`` should identify one execution. Model snapshots are returned as
    paths because their format is space-specific. Use
    :func:`load_multi_run_results` for a collection of executions.
    """
    root = Path(root).expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"Result directory does not exist: {root}")

    globber = root.rglob if recursive else root.glob
    result = ExperimentResults(root=root)

    for name, pattern in _TABULAR_FILES.items():
        paths = sorted(path for path in globber(pattern) if path.is_file())
        if paths:
            result.paths[name] = paths
            result.data[name] = _read_table(paths, name)

    result.model_files = sorted(
        path for path in globber("models_save_*/*.pth") if path.is_file()
    )
    result.log_files = sorted(path for path in globber("*.log") if path.is_file())
    result.yaml_files = sorted(path for path in globber("*.yaml") if path.is_file())
    return result


def load_multi_run_results(root: str | Path) -> MultiRunResults:
    """Discover and independently load executions below ``root``.

    The first directory component below ``root`` is the run identifier. This
    keeps job IDs and result-file directories out of the statistical identity
    of an execution.
    """
    root = Path(root).expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"Result directory does not exist: {root}")

    run_paths: dict[str, Path] = {}
    for pattern in _TABULAR_FILES.values():
        for path in root.rglob(pattern):
            if not path.is_file():
                continue
            relative = path.relative_to(root)
            if len(relative.parts) < 2:
                continue
            run_id = relative.parts[0]
            run_paths[run_id] = root / run_id

    if not run_paths:
        raise FileNotFoundError(
            f"No execution result files found below collection root {root}"
        )

    return MultiRunResults(
        root=root,
        runs={
            run_id: load_results(run_path)
            for run_id, run_path in sorted(run_paths.items())
        },
    )

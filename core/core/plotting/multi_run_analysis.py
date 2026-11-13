"""Statistical tables built from independently loaded experiment runs."""

from __future__ import annotations

from collections.abc import Callable, Iterable

import pandas as pd

from core.plotting.result_loader import MultiRunResults
from core.plotting.success_analysis import (
    cumulative_purpose_achievement_table,
    domain_success_summary,
    purpose_achievement_table,
    purpose_success_summary,
    rewarded_iteration_rate_table,
    trial_success_table,
)


def _require_multi_run(results: MultiRunResults) -> MultiRunResults:
    if not isinstance(results, MultiRunResults):
        raise TypeError(
            "Multi-run analysis requires a MultiRunResults object. "
            "Use load_multi_run_results(...) first."
        )
    return results


def _run_tables(
    results: MultiRunResults,
    generator: Callable,
    **kwargs,
) -> list[pd.DataFrame]:
    results = _require_multi_run(results)
    tables = []
    for run_id, run_results in results.runs.items():
        table = generator(run_results, **kwargs).copy()
        table.insert(0, "run_id", run_id)
        tables.append(table)
    return tables


def _aggregate(
    tables: Iterable[pd.DataFrame],
    *,
    group_columns: list[str],
    value_columns: list[str],
) -> pd.DataFrame:
    frames = [table for table in tables if not table.empty]
    output_columns = [*group_columns, "n_runs"]
    for column in value_columns:
        output_columns.extend(
            [f"median_{column}", f"q1_{column}", f"q3_{column}", f"iqr_{column}"]
        )
    if not frames:
        return pd.DataFrame(columns=output_columns)

    data = pd.concat(frames, ignore_index=True)
    grouped = data.groupby(group_columns, dropna=False, sort=True)
    result = grouped[value_columns].median().rename(
        columns=lambda column: f"median_{column}"
    )
    result = result.join(
        grouped[value_columns].quantile(0.25).rename(
            columns=lambda column: f"q1_{column}"
        )
    )
    result = result.join(
        grouped[value_columns].quantile(0.75).rename(
            columns=lambda column: f"q3_{column}"
        )
    )
    result = result.join(
        grouped[value_columns[0]].count().rename("n_runs")
    )
    result = result.reset_index()
    for column in value_columns:
        result[f"iqr_{column}"] = (
            result[f"q3_{column}"] - result[f"q1_{column}"]
        )
    return result[output_columns]


def multi_trial_success_table(results: MultiRunResults) -> pd.DataFrame:
    """Aggregate trial outcomes across the common trial range.

    The returned DataFrame is trimmed at the largest trial available in every
    run. The cutoff and the largest trial in any run are available through
    ``table.attrs["trimmed_at_trial"]`` and
    ``table.attrs["maximum_available_trial"]``.
    """
    tables = _run_tables(results, trial_success_table)
    if not tables:
        result = pd.DataFrame()
        result.attrs["trimmed_at_trial"] = None
        result.attrs["maximum_available_trial"] = None
        return result

    for table in tables:
        table["success_rate"] = table["success"].astype(int)

    run_maxima = [
        int(table["trial"].max())
        for table in tables
        if not table.empty
    ]
    if not run_maxima:
        result = _aggregate(
            tables,
            group_columns=["trial"],
            value_columns=["success_rate", "iterations", "episode_count"],
        )
        result.attrs["trimmed_at_trial"] = None
        result.attrs["maximum_available_trial"] = None
        return result

    trimmed_at_trial = min(run_maxima)
    maximum_available_trial = max(run_maxima)
    result = _aggregate(
        [
            table[table["trial"] <= trimmed_at_trial].copy()
            for table in tables
        ],
        group_columns=["trial"],
        value_columns=["success_rate", "iterations", "episode_count"],
    )
    result.attrs["trimmed_at_trial"] = trimmed_at_trial
    result.attrs["maximum_available_trial"] = maximum_available_trial
    return result


def multi_purpose_achievement_table(results: MultiRunResults) -> pd.DataFrame:
    """Return purpose achievement events independently resolved per run."""
    tables = _run_tables(results, purpose_achievement_table)
    columns = ["run_id", "iteration", "domain", "goal", "purpose", "achieved"]
    return (
        pd.concat(tables, ignore_index=True)[columns]
        if tables
        else pd.DataFrame(columns=columns)
    )


def multi_purpose_success_summary(
    results: MultiRunResults,
    *,
    by_domain: bool = True,
    top_level_only: bool = True,
) -> pd.DataFrame:
    """Aggregate per-run purpose achievement rates.

    The default summarizes top-level purpose achievements only. Set
    ``top_level_only=False`` to include downstream goal rewards.
    """
    tables = _run_tables(
        results,
        purpose_success_summary,
        by_domain=by_domain,
        top_level_only=top_level_only,
    )
    group_columns = ["domain", "purpose"] if by_domain else ["purpose"]
    return _aggregate(
        tables,
        group_columns=group_columns,
        value_columns=[
            "attempts",
            "achievements",
            "achievement_rate",
            "first_achievement_iteration",
        ],
    )


def multi_domain_success_summary(results: MultiRunResults) -> pd.DataFrame:
    """Aggregate per-run domain achievement rates."""
    tables = _run_tables(results, domain_success_summary)
    return _aggregate(
        tables,
        group_columns=["domain"],
        value_columns=["attempts", "achievements", "achievement_rate"],
    )


def multi_cumulative_purpose_achievement_table(
    results: MultiRunResults,
    *,
    top_level_only: bool = True,
) -> pd.DataFrame:
    """Aggregate cumulative purpose achievements across runs.

    The default includes top-level goals only. Set ``top_level_only=False``
    to include downstream goals explicitly.
    """
    tables = _run_tables(
        results,
        cumulative_purpose_achievement_table,
        top_level_only=top_level_only,
    )
    return _aggregate(
        tables,
        group_columns=["iteration", "domain", "purpose"],
        value_columns=["cumulative_achievements"],
    )


def multi_rewarded_iteration_rate_table(
    results: MultiRunResults,
    *,
    window: int = 20,
) -> pd.DataFrame:
    """Aggregate rolling rewarded-iteration rates calculated per run."""
    tables = _run_tables(
        results,
        rewarded_iteration_rate_table,
        window=window,
    )
    return _aggregate(
        tables,
        group_columns=["iteration", "domain", "purpose"],
        value_columns=["rewarded_iteration_rate"],
    )

"""Execution-success tables built from an :class:`ExperimentResults` bundle."""

from __future__ import annotations

from collections import deque
from typing import Any

import pandas as pd

from core.plotting.goal_analysis import goal_achievement_table
from core.plotting.result_loader import ExperimentResults


def _require_results(results: ExperimentResults) -> ExperimentResults:
    if not isinstance(results, ExperimentResults):
        raise TypeError(
            "Success analysis requires an ExperimentResults object. "
            "Use load_results(...) first."
        )
    return results


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"true", "1"}:
        return True
    if normalized in {"false", "0"}:
        return False
    raise ValueError(f"Invalid trial success value: {value!r}")


def trial_success_table(results: ExperimentResults) -> pd.DataFrame:
    """Return one normalized row per terminal execution trial."""
    trials = _require_results(results).table("trials").copy()
    required = {"Iteration", "Trial", "Iterations", "EpisodeCount", "Success"}
    missing = required - set(trials.columns)
    if missing:
        raise ValueError(f"Trials table is missing columns: {sorted(missing)}")
    result = trials[
        ["Iteration", "Trial", "Iterations", "EpisodeCount", "Success"]
    ].copy()
    result["Success"] = result["Success"].map(_parse_bool)
    return result.rename(
        columns={
            "Iteration": "iteration",
            "Trial": "trial",
            "Iterations": "iterations",
            "EpisodeCount": "episode_count",
            "Success": "success",
        }
    )[
        ["iteration", "trial", "iterations", "episode_count", "success"]
    ]


def goal_purpose_lookup(results: ExperimentResults) -> pd.DataFrame:
    """Return the graph-derived lookup from every goal to its drive purpose."""
    neighbors = _require_results(results).table("neighbors_full")
    required = {"NodeType", "NodeName", "NeighborName", "NeighborType"}
    missing = required - set(neighbors.columns)
    if missing:
        raise ValueError(f"Neighbors table is missing columns: {sorted(missing)}")

    graph: dict[str, list[tuple[str, str]]] = {}
    for row in neighbors.itertuples(index=False):
        graph.setdefault(str(row.NodeName), []).append(
            (str(row.NeighborName), str(row.NeighborType))
        )

    goals = set(
        neighbors.loc[neighbors["NodeType"].eq("Goal"), "NodeName"].astype(str)
    )
    drives = set(
        neighbors.loc[neighbors["NodeType"].eq("Drive"), "NodeName"].astype(str)
    )
    lookup: list[dict[str, str | None]] = []
    lookup.extend({"goal": drive, "purpose": drive} for drive in drives)
    for goal in goals:
        pending = deque([goal])
        visited = {goal}
        purpose = None
        while pending and purpose is None:
            current = pending.popleft()
            for neighbor, neighbor_type in graph.get(current, []):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                if neighbor_type == "Drive":
                    purpose = neighbor
                    break
                if neighbor_type == "Goal":
                    pending.append(neighbor)
        lookup.append({"goal": goal, "purpose": purpose})

    return pd.DataFrame(lookup, columns=["goal", "purpose"])


def purpose_achievement_table(results: ExperimentResults) -> pd.DataFrame:
    """Return goal achievement events annotated with their drive purpose.

    Goal occurrences and achievement tallies come from the existing
    :func:`goal_achievement_table`. The neighbor graph is consulted once to
    build a goal-to-purpose lookup; it is not traversed per goodness row.
    """
    results = _require_results(results)
    achievements = goal_achievement_table(results.table("goodness"))
    lookup = goal_purpose_lookup(results)
    result = achievements.merge(lookup, on="goal", how="left", validate="many_to_one")
    return result[
        ["iteration", "domain", "goal", "purpose", "achieved"]
    ]


def purpose_success_summary(
    results: ExperimentResults,
    *,
    by_domain: bool = True,
) -> pd.DataFrame:
    """Summarize goal achievement rates by resolved drive purpose."""
    events = purpose_achievement_table(_require_results(results))
    group_columns = ["purpose"]
    if by_domain:
        group_columns.insert(0, "domain")
    if events.empty:
        return pd.DataFrame(
            columns=[
                *group_columns,
                "attempts",
                "achievements",
                "achievement_rate",
                "first_achievement_iteration",
            ]
        )

    summary = (
        events.groupby(group_columns, dropna=False, sort=False)["achieved"]
        .agg(attempts="size", achievements="sum")
        .reset_index()
    )
    summary["achievement_rate"] = summary["achievements"] / summary["attempts"]
    first = (
        events[events["achieved"]]
        .groupby(group_columns, dropna=False, sort=False)["iteration"]
        .min()
        .rename("first_achievement_iteration")
    )
    return summary.merge(first, on=group_columns, how="left")


def domain_success_summary(results: ExperimentResults) -> pd.DataFrame:
    """Summarize all goal achievement events by execution domain."""
    events = purpose_achievement_table(_require_results(results))
    if events.empty:
        return pd.DataFrame(
            columns=["domain", "attempts", "achievements", "achievement_rate"]
        )
    summary = (
        events.groupby("domain", sort=False)["achieved"]
        .agg(attempts="size", achievements="sum")
        .reset_index()
    )
    summary["achievement_rate"] = summary["achievements"] / summary["attempts"]
    return summary


def cumulative_purpose_achievement_table(
    results: ExperimentResults,
) -> pd.DataFrame:
    """Return cumulative achievements by purpose, domain, and iteration."""
    events = purpose_achievement_table(_require_results(results))
    columns = [
        "iteration",
        "domain",
        "purpose",
        "cumulative_achievements",
    ]
    if events.empty:
        return pd.DataFrame(columns=columns)

    events = events.copy()
    events["iteration"] = pd.to_numeric(events["iteration"], errors="raise")
    counts = (
        events.assign(achievement=events["achieved"].astype(int))
        .groupby(["iteration", "domain", "purpose"], dropna=False, sort=True)[
            "achievement"
        ]
        .sum()
        .reset_index()
    )
    counts["cumulative_achievements"] = counts.groupby(
        ["domain", "purpose"], dropna=False, sort=False
    )["achievement"].cumsum()
    return counts[columns]


def rewarded_iteration_rate_table(
    results: ExperimentResults,
    *,
    window: int = 20,
) -> pd.DataFrame:
    """Return rolling rewarded-iteration rates by domain and purpose.

    An iteration is rewarded for a domain/purpose pair when at least one goal
    associated with that purpose is achieved during that iteration and domain.
    The rate is calculated over the latest ``window`` observed iterations for
    that pair. The first rows use all available observations until the window
    is full.
    """
    if window < 1:
        raise ValueError("window must be at least 1")

    events = purpose_achievement_table(_require_results(results))
    columns = [
        "iteration",
        "domain",
        "purpose",
        "rewarded_iteration",
        "iterations",
        "rewarded_iterations",
        "rewarded_iteration_rate",
    ]
    if events.empty:
        return pd.DataFrame(columns=columns)

    iteration_events = (
        events.groupby(
            ["iteration", "domain", "purpose"],
            dropna=False,
            sort=True,
        )["achieved"]
        .any()
        .rename("rewarded_iteration")
        .reset_index()
    )
    iteration_events["iteration"] = pd.to_numeric(
        iteration_events["iteration"],
        errors="raise",
    )
    iteration_events = iteration_events.sort_values(
        ["domain", "purpose", "iteration"],
        kind="stable",
    )
    group_columns = ["domain", "purpose"]
    iteration_events["rewarded_iteration"] = (
        iteration_events["rewarded_iteration"].astype(int)
    )
    iteration_events["iterations"] = iteration_events.groupby(
        group_columns,
        dropna=False,
        sort=False,
    ).cumcount() + 1
    iteration_events["rewarded_iterations"] = iteration_events.groupby(
        group_columns,
        dropna=False,
        sort=False,
    )["rewarded_iteration"].transform(
        lambda values: values.rolling(window=window, min_periods=window).sum()
    )
    iteration_events["iterations"] = iteration_events.groupby(
        group_columns,
        dropna=False,
        sort=False,
    )["rewarded_iteration"].transform(
        lambda values: values.rolling(window=window, min_periods=1).count()
    )
    iteration_events["rewarded_iteration_rate"] = (
        iteration_events["rewarded_iterations"]
        / iteration_events["iterations"]
    )
    return iteration_events[columns]


analyze_trial_success = trial_success_table
analyze_purpose_success = purpose_achievement_table

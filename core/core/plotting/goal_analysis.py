"""Analysis helpers for goal achievements recorded in goodness tables."""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Any, Iterable

import pandas as pd

from core.plotting.result_loader import ExperimentResults


_REQUIRED_COLUMNS = {"Iteration", "World", "Goal reward list"}
_OUTPUT_COLUMNS = ["goal", "domain", "iteration", "achieved"]
_CUMULATIVE_COLUMN = "cumulative_achievements"


def _load_goodness(
    source: ExperimentResults | str | Path | pd.DataFrame,
) -> pd.DataFrame:
    if isinstance(source, ExperimentResults):
        return source.table("goodness").copy()
    if isinstance(source, pd.DataFrame):
        return source.copy()

    path = Path(source).expanduser()
    if path.is_dir():
        paths = sorted(path.glob("goodness_*.txt"))
        if not paths:
            raise FileNotFoundError(f"No goodness_*.txt files found in {path}")
    elif path.is_file():
        paths = [path]
    else:
        raise FileNotFoundError(f"Goodness file or directory does not exist: {path}")

    frames = [
        pd.read_csv(file_path, sep="\t", comment="#")
        for file_path in paths
        if file_path.stat().st_size > 0
    ]
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def _parse_goal_rewards(value: Any, row_number: int) -> dict[str, Any]:
    if isinstance(value, dict):
        rewards = value
    elif isinstance(value, str):
        try:
            rewards = ast.literal_eval(value)
        except (SyntaxError, ValueError) as error:
            raise ValueError(
                f"Invalid goal reward list at input row {row_number}"
            ) from error
    else:
        raise TypeError(
            f"Goal reward list at input row {row_number} must be a dictionary"
        )

    if not isinstance(rewards, dict):
        raise TypeError(
            f"Goal reward list at input row {row_number} must be a dictionary"
        )
    return rewards


def goal_achievement_table(
    source: ExperimentResults | str | Path | pd.DataFrame,
    *,
    tally_by_domain: bool = False,
    exclude_goals: Iterable[str] | None = None,
) -> pd.DataFrame:
    """Return one event row for every goal occurrence.

    ``source`` may be one ``goodness_x.txt`` file, a directory containing
    goodness files, or a DataFrame with the goodness table columns. A goal is
    considered achieved when its reward is numerically equal to ``1``. The
    ``achieved`` column records both successful and unsuccessful occurrences.

    When ``tally_by_domain`` is true, ``effective_iteration`` counts rows
    independently within each ``World`` (domain). Otherwise, the result only
    contains the raw iteration values from the input. A DataFrame returned by
    this function can be passed back in as ``source``.
    """
    excluded = {str(goal) for goal in (exclude_goals or ())}

    if isinstance(source, pd.DataFrame) and set(_OUTPUT_COLUMNS).issubset(
        source.columns
    ):
        result = source.copy()
        if excluded:
            result = result[~result["goal"].astype(str).isin(excluded)]
        if tally_by_domain and "effective_iteration" not in result:
            result["effective_iteration"] = (
                result.groupby("domain", sort=False).cumcount() + 1
            )
        elif not tally_by_domain:
            result = result.drop(columns=["effective_iteration"], errors="ignore")
        columns = _OUTPUT_COLUMNS + (
            ["effective_iteration"] if tally_by_domain else []
        )
        return result[columns]

    goodness = _load_goodness(source)
    missing = _REQUIRED_COLUMNS - set(goodness.columns)
    if missing:
        raise ValueError(f"Goodness table is missing columns: {sorted(missing)}")

    rows: list[dict[str, Any]] = []
    domain_iterations: dict[Any, int] = {}
    for row_number, row in goodness.iterrows():
        domain = row["World"]
        domain_iterations[domain] = domain_iterations.get(domain, 0) + 1
        rewards = _parse_goal_rewards(row["Goal reward list"], row_number)
        for goal, reward in rewards.items():
            if str(goal) in excluded:
                continue
            try:
                achieved = float(reward) == 1.0
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"Invalid reward for goal {goal!r} at input row {row_number}"
                ) from error
            rows.append({
                "goal": goal,
                "domain": domain,
                "iteration": row["Iteration"],
                "achieved": achieved,
                "effective_iteration": domain_iterations[domain],
            })

    columns = _OUTPUT_COLUMNS + (["effective_iteration"] if tally_by_domain else [])
    result = pd.DataFrame(rows, columns=columns)
    return result


def cumulative_goal_achievement_table(
    source: ExperimentResults | str | Path | pd.DataFrame,
    *,
    tally_by_domain: bool = False,
    exclude_goals: Iterable[str] | None = None,
) -> pd.DataFrame:
    """Return a cumulative goal-achievement time series.

    The result has one row per goal and x-axis value and can be plotted
    directly. Its x-axis column is ``iteration`` or ``effective_iteration``,
    its y-axis column is ``cumulative_achievements``, and ``goal`` identifies
    each independent series. Achievements are counted across domains.
    """
    achievements = goal_achievement_table(
        source,
        tally_by_domain=tally_by_domain,
        exclude_goals=exclude_goals,
    )
    x_column = "effective_iteration" if tally_by_domain else "iteration"
    columns = ["goal", x_column, _CUMULATIVE_COLUMN]
    if achievements.empty:
        return pd.DataFrame(columns=columns)

    achievements = achievements.assign(
        **{x_column: pd.to_numeric(achievements[x_column], errors="raise")}
    )
    counts = (
        achievements.assign(
            achievement=achievements["achieved"].astype(int)
        )
        .groupby(["goal", x_column], sort=True)["achievement"]
        .sum()
        .rename("achievements")
        .reset_index()
    )
    counts[_CUMULATIVE_COLUMN] = counts.groupby("goal")["achievements"].cumsum()
    return counts[columns]


def goal_discovery_table(
    source: ExperimentResults | str | Path | pd.DataFrame,
    *,
    exclude_goals: Iterable[str] | None = None,
) -> pd.DataFrame:
    """Return the first iteration in which each goal is observed.

    A goal is considered discovered when it appears in the goodness reward
    dictionary, regardless of whether its reward is zero or one. Discovery is
    global for each goal; the domain column records where it was first seen.
    """
    achievements = goal_achievement_table(
        source,
        exclude_goals=exclude_goals,
    )
    if achievements.empty:
        return pd.DataFrame(columns=["goal", "domain", "iteration"])
    discoveries = (
        achievements.sort_values("iteration", kind="stable")
        .drop_duplicates("goal", keep="first")
        [["goal", "domain", "iteration"]]
        .reset_index(drop=True)
    )
    return discoveries


def cumulative_goal_discovery_table(
    source: ExperimentResults | str | Path | pd.DataFrame,
    *,
    by_domain: bool = False,
    exclude_goals: Iterable[str] | None = None,
) -> pd.DataFrame:
    """Return cumulative counts of newly observed goals by iteration.

    With ``by_domain=False``, each goal is counted once across the experiment.
    With ``by_domain=True``, a goal is counted once independently for every
    domain where it is observed.
    """
    achievements = goal_achievement_table(
        source,
        exclude_goals=exclude_goals,
    )
    columns = [
        "iteration",
        "goals_discovered",
        "cumulative_goals_discovered",
    ]
    group_columns: list[str] = []
    if by_domain:
        columns.insert(1, "domain")
        group_columns = ["domain"]
    if achievements.empty:
        return pd.DataFrame(columns=columns)

    keys = group_columns + ["goal"]
    discoveries = (
        achievements.sort_values("iteration", kind="stable")
        .drop_duplicates(keys, keep="first")
    )
    counts = (
        discoveries.groupby(group_columns + ["iteration"], sort=True)
        ["goal"]
        .nunique()
        .rename("goals_discovered")
        .reset_index()
    )
    sort_columns = group_columns + ["iteration"]
    counts = counts.sort_values(sort_columns, kind="stable")
    if group_columns:
        counts["cumulative_goals_discovered"] = counts.groupby(
            group_columns,
            sort=False,
        )["goals_discovered"].cumsum()
    else:
        counts["cumulative_goals_discovered"] = counts["goals_discovered"].cumsum()
    return counts[columns]


analyze_goal_achievements = goal_achievement_table
analyze_goal_discoveries = goal_discovery_table

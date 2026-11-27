from core.plotting.multi_run_analysis import (
    multi_cumulative_goal_discovery_table,
    multi_trial_success_table,
)
from core.plotting.result_loader import load_multi_run_results


def _write_trials(run_root, count):
    results_dir = run_root / "job_1" / "results_txt"
    results_dir.mkdir(parents=True)
    rows = ["Iteration\tTrial\tIterations\tEpisodeCount\tSuccess"]
    rows.extend(
        f"{trial * 10}\t{trial}\t10\t9\tTrue"
        for trial in range(count)
    )
    (results_dir / "trials_0.txt").write_text("\n".join(rows))


def test_multi_run_loading_is_lazy_and_analysis_releases_tables(tmp_path):
    _write_trials(tmp_path / "run_short", 3)
    _write_trials(tmp_path / "run_long", 5)

    results = load_multi_run_results(tmp_path)

    assert set(results.runs) == {"run_short", "run_long"}
    assert all(run.lazy for run in results.runs.values())
    assert all(not run.data for run in results.runs.values())
    assert "trials" in results.run("run_short").available_tables

    trials = results.run("run_short").table("trials")
    assert len(trials) == 3
    assert "trials" in results.run("run_short").data
    results.run("run_short").release_table("trials")
    assert "trials" not in results.run("run_short").data

    summary = multi_trial_success_table(results)

    assert summary["trial"].tolist() == [0, 1, 2]
    assert summary.attrs["trimmed_at_trial"] == 2
    assert summary.attrs["maximum_available_trial"] == 4
    assert all(not run.data for run in results.runs.values())


def test_multi_goal_discovery_carries_counts_between_run_iterations(tmp_path):
    discoveries = {
        "run_1": [(10, "goal_a"), (40, "goal_b")],
        "run_2": [(20, "goal_c"), (50, "goal_d")],
        "run_3": [(30, "goal_e"), (60, "goal_f")],
    }
    for run_id, run_discoveries in discoveries.items():
        results_dir = tmp_path / run_id / "job_1" / "results_txt"
        results_dir.mkdir(parents=True)
        rows = ["Iteration\tWorld\tGoal reward list"]
        rows.extend(
            f"{iteration}\tBARTENDER\t{{'{goal}': 0}}"
            for iteration, goal in run_discoveries
        )
        (results_dir / "goodness_0.txt").write_text("\n".join(rows))

    results = load_multi_run_results(tmp_path)
    summary = multi_cumulative_goal_discovery_table(results)

    assert summary["iteration"].tolist() == [10, 20, 30, 40, 50, 60]
    assert summary["median_cumulative_goals_discovered"].tolist() == [
        0,
        1,
        1,
        1,
        2,
        2,
    ]

    domain_summary = multi_cumulative_goal_discovery_table(
        results,
        by_domain=True,
    )
    assert domain_summary["median_cumulative_goals_discovered"].tolist() == [
        0,
        1,
        1,
        1,
        2,
        2,
    ]

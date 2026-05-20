# -*- coding: utf-8 -*-
"""Generate simple publication-ready RTF figures from benchmark results.

Default use from the Webots project root:

    python scripts/generate_rtf_figures.py

Input:
    Extracted data/performance/all_scenarios_rtf_timeseries.csv

Outputs:
    Extracted data/performance/rtf_figures/fig_rtf_time_series.png
    Extracted data/performance/rtf_figures/fig_rtf_time_series.pdf
    Extracted data/performance/rtf_figures/fig_rtf_boxplot.png
    Extracted data/performance/rtf_figures/fig_rtf_boxplot.pdf

The script intentionally does not generate LaTeX code, ZIP packages, or
additional report files. It only produces the two figures needed for the paper. Figure settings such as the y-axis range are fixed in this file so the command remains simple and the output remains consistent.

Python 3.7.9 compatible.
"""

from __future__ import print_function

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.transforms import blended_transform_factory

VERSION = "v3-2026-05-17-simple-rtf-figures-fixed-axis-py379"

DEFAULT_INPUT = Path("Extracted data") / "performance" / "all_scenarios_rtf_timeseries.csv"
DEFAULT_OUTPUT_DIR = Path("Extracted data") / "performance" / "rtf_figures"

# Figure settings are intentionally fixed here rather than exposed as
# command-line options. This keeps the plotting command simple and makes
# repeated paper figures visually consistent.
PLOT_Y_MIN = 0.0
PLOT_Y_MAX = 2.5
PLOT_Y_TICK_STEP = 0.5
RTF_NEAR_LOW = 0.90
RTF_NEAR_HIGH = 1.10
FIGURE_DPI = 320
FIGURE_FONT_SIZE = 11
FIGURE_WIDTH = 11.0
TREND_FIGURE_HEIGHT = 4.4
BOX_FIGURE_HEIGHT = 3.8
MAX_ANNOTATED_LINES = 8
ANNOTATION_FRACTION = 0.82
ANNOTATION_Y_OFFSET = 0.035


CLASS_COLOUR = {
    "Above real time": "tab:red",
    "Near real time": "tab:green",
    "Below real time": "tab:blue",
}


def find_project_root(start=None):
    """Return the project root containing main.py and worlds/."""
    start_path = Path(start or Path.cwd()).resolve()
    candidates = [start_path] + list(start_path.parents)
    try:
        script_path = Path(__file__).resolve()
        candidates = [script_path.parent] + list(script_path.parents) + candidates
    except Exception:
        pass

    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        if (candidate / "main.py").exists() and (candidate / "worlds").is_dir():
            return candidate
    return Path.cwd().resolve()


def resolve_project_path(path_value, root):
    """Resolve a path relative to the project root unless it is absolute."""
    path = Path(path_value)
    if path.is_absolute():
        return path
    return (root / path).resolve()


def read_benchmark_file(path):
    """Read the benchmark result table from CSV or Excel."""
    suffix = path.suffix.lower()
    if suffix in (".xlsx", ".xls"):
        return pd.read_excel(str(path))
    return pd.read_csv(str(path))


def validate_and_prepare(df, metric_col):
    """Validate required columns and return sorted numeric rows."""
    required = set(["scenario_id", "measurement_sim_time", metric_col])
    missing = required - set(df.columns)
    if missing:
        raise ValueError("Missing required columns: {}".format(sorted(missing)))

    df = df.copy()

    # CAV count may be stored under either name depending on benchmark version.
    if "n_surrounding_cavs" not in df.columns:
        if "target_surrounding_cavs" in df.columns:
            df["n_surrounding_cavs"] = df["target_surrounding_cavs"]
        else:
            raise ValueError("Missing 'n_surrounding_cavs' or 'target_surrounding_cavs'.")

    if "measurement_step_index" not in df.columns:
        if "sample_index" in df.columns:
            df["measurement_step_index"] = df["sample_index"]
        else:
            df["measurement_step_index"] = np.arange(1, len(df) + 1)

    df[metric_col] = pd.to_numeric(df[metric_col], errors="coerce")
    df["measurement_sim_time"] = pd.to_numeric(df["measurement_sim_time"], errors="coerce")
    df["measurement_step_index"] = pd.to_numeric(df["measurement_step_index"], errors="coerce")
    df["n_surrounding_cavs"] = pd.to_numeric(df["n_surrounding_cavs"], errors="coerce")

    df = df[np.isfinite(df[metric_col])].copy()
    df = df[np.isfinite(df["measurement_sim_time"])].copy()
    df = df[np.isfinite(df["n_surrounding_cavs"])].copy()
    df = df.sort_values(["n_surrounding_cavs", "scenario_id", "measurement_step_index"]).copy()
    return df


def classify_rtf(value, near_low, near_high):
    """Classify the final scenario RTF relative to the near-real-time band."""
    if value > near_high:
        return "Above real time"
    if value < near_low:
        return "Below real time"
    return "Near real time"


def build_scenario_summary(df, metric_col, near_low, near_high):
    """Summarise each scenario for plotting and annotation."""
    rows = []
    for scenario_id, group in df.groupby("scenario_id"):
        g = group.sort_values("measurement_step_index")
        final_rtf = float(g[metric_col].iloc[-1])
        n_cavs = int(round(float(g["n_surrounding_cavs"].iloc[0])))
        rows.append({
            "scenario_id": scenario_id,
            "n_surrounding_cavs": n_cavs,
            "final_real_time_factor": final_rtf,
            "abs_distance_from_1": abs(final_rtf - 1.0),
            "performance_class": classify_rtf(final_rtf, near_low, near_high),
        })
    return pd.DataFrame(rows).sort_values(["n_surrounding_cavs", "scenario_id"]).reset_index(drop=True)


def configure_matplotlib(font_size):
    plt.rcParams.update({
        "font.size": font_size,
        "axes.titlesize": font_size + 2,
        "axes.labelsize": font_size + 1,
        "xtick.labelsize": font_size,
        "ytick.labelsize": font_size,
        "legend.fontsize": font_size,
    })


def apply_rtf_axis(ax, y_min, y_max, y_tick_step, near_low, near_high):
    """Apply common RTF axis styling and the near-real-time band."""
    ax.set_ylim(y_min, y_max)
    ax.set_yticks(np.arange(y_min, y_max + 0.0001, y_tick_step))
    ax.axhspan(near_low, near_high, color="0.90", alpha=0.60, zorder=0)
    ax.axhline(1.0, linestyle="--", linewidth=1.1, color="black", zorder=2)
    ax.axhline(near_high, linestyle=":", linewidth=0.9, color="black", zorder=2)
    ax.axhline(near_low, linestyle=":", linewidth=0.9, color="black", zorder=2)

    trans = blended_transform_factory(ax.transAxes, ax.transData)
    ax.text(1.006, near_high, "{:.1f}".format(near_high), transform=trans,
            ha="left", va="center", fontsize=8)
    ax.text(1.006, near_low, "{:.1f}".format(near_low), transform=trans,
            ha="left", va="center", fontsize=8)
    ax.text(0.985, 1.0, "{:.1f}--{:.1f} band".format(near_low, near_high),
            transform=trans, ha="right", va="center", fontsize=8,
            bbox={"boxstyle": "round,pad=0.18", "fc": "white", "ec": "0.55", "lw": 0.4, "alpha": 0.90})


def class_legend_handles():
    return [
        Line2D([0], [0], color=CLASS_COLOUR["Above real time"], lw=2.0, label="Above real time"),
        Line2D([0], [0], color=CLASS_COLOUR["Near real time"], lw=2.0, label="Near real time"),
        Line2D([0], [0], color=CLASS_COLOUR["Below real time"], lw=2.0, label="Below real time"),
    ]


def choose_annotated_cavs(summary, explicit=None, max_labels=8):
    """Choose CAV counts to label on the trend figure."""
    if explicit:
        return set([int(x) for x in explicit])

    cavs = sorted(summary["n_surrounding_cavs"].astype(int).unique().tolist())
    if len(cavs) <= max_labels:
        return set(cavs)

    positions = np.linspace(0, len(cavs) - 1, max_labels).round().astype(int)
    selected = set([cavs[int(i)] for i in positions])

    # Always annotate the scenario closest to real time.
    best = summary.sort_values("abs_distance_from_1").iloc[0]
    selected.add(int(best["n_surrounding_cavs"]))
    return selected


def save_figure(fig, output_base, dpi):
    """Save each figure as PNG and PDF."""
    fig.savefig(str(output_base) + ".png", dpi=dpi, bbox_inches="tight")
    fig.savefig(str(output_base) + ".pdf", bbox_inches="tight")


def plot_time_series(df, summary, args):
    """Create the RTF time-series plot."""
    fig, ax = plt.subplots(figsize=(args.figure_width, args.trend_height))
    annotated_cavs = choose_annotated_cavs(summary, args.annotate_cavs, args.max_annotated_lines)

    for _, row in summary.iterrows():
        scenario_id = row["scenario_id"]
        n_cavs = int(row["n_surrounding_cavs"])
        colour = CLASS_COLOUR.get(row["performance_class"], "0.35")
        highlight = n_cavs in annotated_cavs
        g = df[df["scenario_id"] == scenario_id].sort_values("measurement_sim_time")

        ax.plot(
            g["measurement_sim_time"],
            g[args.metric],
            color=colour,
            linewidth=2.0 if highlight else 1.0,
            alpha=0.90 if highlight else 0.30,
            zorder=3 if highlight else 1,
        )

        if highlight and len(g) > 0:
            # Label toward the right-hand side of the curve to keep the plot readable.
            idx = int(max(0, min(len(g) - 1, round(args.annotation_fraction * (len(g) - 1)))))
            x0 = float(g["measurement_sim_time"].iloc[idx])
            y0 = float(g[args.metric].iloc[idx])
            ax.annotate(
                str(n_cavs),
                xy=(x0, y0),
                xytext=(x0, y0 + args.annotation_y_offset),
                fontsize=8,
                fontweight="bold",
                color=colour,
                ha="center",
                va="center",
                arrowprops={"arrowstyle": "-", "color": colour, "lw": 0.6, "alpha": 0.70},
                bbox={"boxstyle": "round,pad=0.14", "fc": "white", "ec": colour, "lw": 0.35, "alpha": 0.92},
                zorder=5,
                clip_on=False,
            )

    apply_rtf_axis(ax, args.y_min, args.y_max, args.y_tick_step, args.near_low, args.near_high)
    ax.set_xlabel("Measurement simulation time (s)")
    ax.set_ylabel("Real-time factor")
    ax.set_title("Real-time factor by benchmark scenario")
    ax.grid(axis="y", linewidth=0.4, alpha=0.35)
    ax.legend(handles=class_legend_handles(), loc="upper right", frameon=True)
    plt.tight_layout()
    save_figure(fig, args.output_dir / "fig_rtf_time_series", args.dpi)
    plt.close(fig)


def plot_boxplot(df, summary, args):
    """Create the RTF distribution boxplot by surrounding CAV count."""
    fig, ax = plt.subplots(figsize=(args.figure_width, args.box_height))

    data = []
    labels = []
    colours = []
    for _, row in summary.iterrows():
        scenario_id = row["scenario_id"]
        g = df[df["scenario_id"] == scenario_id]
        data.append(g[args.metric].dropna().values)
        labels.append(str(int(row["n_surrounding_cavs"])))
        colours.append(CLASS_COLOUR.get(row["performance_class"], "0.6"))

    try:
        bp = ax.boxplot(data, tick_labels=labels, showmeans=True, patch_artist=True, widths=0.65)
    except TypeError:
        # Matplotlib versions older than 3.9 use `labels` instead of `tick_labels`.
        bp = ax.boxplot(data, labels=labels, showmeans=True, patch_artist=True, widths=0.65)

    for box, colour in zip(bp["boxes"], colours):
        box.set_facecolor(colour)
        box.set_alpha(0.28)
        box.set_edgecolor(colour)
        box.set_linewidth(1.1)
    for median in bp["medians"]:
        median.set_color("black")
        median.set_linewidth(1.25)
    for whisker in bp["whiskers"]:
        whisker.set_linewidth(0.9)
    for cap in bp["caps"]:
        cap.set_linewidth(0.9)
    for mean in bp["means"]:
        mean.set_marker("o")
        mean.set_markersize(4.2)

    apply_rtf_axis(ax, args.y_min, args.y_max, args.y_tick_step, args.near_low, args.near_high)
    ax.set_xlabel("Number of surrounding CAVs")
    ax.set_ylabel("Real-time factor")
    ax.set_title("Real-time-factor distribution by surrounding CAV count")
    ax.grid(axis="y", linewidth=0.4, alpha=0.35)
    plt.tight_layout()
    save_figure(fig, args.output_dir / "fig_rtf_boxplot", args.dpi)
    plt.close(fig)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Generate only the two RTF benchmark figures: time series and boxplot.")
    parser.add_argument("--version", action="store_true")
    parser.add_argument("--input", default=str(DEFAULT_INPUT),
                        help="Input benchmark CSV/XLSX. Default: Extracted data/performance/all_scenarios_rtf_timeseries.csv")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR),
                        help="Output directory. Default: Extracted data/performance/rtf_figures")
    parser.add_argument("--metric", default="real_time_factor", help="Metric column to plot. Default: real_time_factor")
    parser.add_argument("--annotate-cavs", nargs="*", type=int, default=None,
                        help="Optional explicit CAV counts to annotate on the time-series plot.")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.version:
        print(VERSION)
        return 0

    # Fixed figure settings used by the plotting functions.
    args.near_low = RTF_NEAR_LOW
    args.near_high = RTF_NEAR_HIGH
    args.y_min = PLOT_Y_MIN
    args.y_max = PLOT_Y_MAX
    args.y_tick_step = PLOT_Y_TICK_STEP
    args.dpi = FIGURE_DPI
    args.font_size = FIGURE_FONT_SIZE
    args.figure_width = FIGURE_WIDTH
    args.trend_height = TREND_FIGURE_HEIGHT
    args.box_height = BOX_FIGURE_HEIGHT
    args.max_annotated_lines = MAX_ANNOTATED_LINES
    args.annotation_fraction = ANNOTATION_FRACTION
    args.annotation_y_offset = ANNOTATION_Y_OFFSET

    project_root = find_project_root()
    input_path = resolve_project_path(args.input, project_root)
    output_dir = resolve_project_path(args.output_dir, project_root)
    args.output_dir = output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    if not input_path.exists():
        raise FileNotFoundError("Benchmark file not found: {}".format(input_path))

    print("[rtf-figures] Project root: {}".format(project_root))
    print("[rtf-figures] Input: {}".format(input_path))
    print("[rtf-figures] Output directory: {}".format(output_dir))

    df = read_benchmark_file(input_path)
    df = validate_and_prepare(df, args.metric)
    summary = build_scenario_summary(df, args.metric, args.near_low, args.near_high)

    configure_matplotlib(args.font_size)
    plot_time_series(df, summary, args)
    plot_boxplot(df, summary, args)

    best = summary.sort_values("abs_distance_from_1").iloc[0]
    print("[rtf-figures] Wrote: {}".format(output_dir / "fig_rtf_time_series.png"))
    print("[rtf-figures] Wrote: {}".format(output_dir / "fig_rtf_boxplot.png"))
    print("[rtf-figures] Closest-to-real-time scenario: {} ({} CAVs, final RTF={:.4f})".format(
        best["scenario_id"], int(best["n_surrounding_cavs"]), float(best["final_real_time_factor"])
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

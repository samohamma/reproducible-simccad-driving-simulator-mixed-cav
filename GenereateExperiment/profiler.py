# -*- coding: utf-8 -*-
"""Webots node emitter for the performance profiling supervisor."""

import json


def _quote(value):
    return json.dumps(str(value))


def emit_performance_profiler_node(
    scenario_id="scenario",
    duration=0.0,
    wall_duration=60.0,
    sample_period=1.0,
    output_dir=None,
    metadata_path=None,
):
    """Emit a hidden Supervisor robot that records simulation real-time factor."""
    args = [
        "--scenario-id", scenario_id,
        "--duration", str(duration),
        "--wall-duration", str(wall_duration),
        "--sample-period", str(sample_period),
    ]
    if output_dir:
        args.extend(["--output-dir", output_dir])
    if metadata_path:
        args.extend(["--metadata", metadata_path])

    arg_lines = "\n    ".join(_quote(a) for a in args)

    return f"""Robot {{
  translation 0 -100 0
  name "performance_profiler_{scenario_id}"
  controller "performance_profiler"
  controllerArgs [
    {arg_lines}
  ]
  supervisor TRUE
}}"""

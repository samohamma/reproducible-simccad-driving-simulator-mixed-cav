# `scripts/`

This folder contains command-line utilities used after world generation.

## Files

| File | Purpose |
|---|---|
| `run_existing_worlds_benchmark.py` | Runs generated Webots `.wbt` worlds, temporarily injects the performance profiler, records real-time-factor metrics, and restores each world. |
| `generate_rtf_figures.py` | Reads benchmark results and generates the two real-time-factor figures used for the paper. |

## Typical usage

From the project root:

```bash
python main.py --benchmark --mode run
python scripts/generate_rtf_figures.py
```

For a short test:

```bash
python main.py --benchmark --mode run --max-scenarios 1 --max-worlds 1
```

The benchmark runner uses Webots `run` mode, corresponding to Ctrl+3.

# Sequential and Parallel PRM Motion Planning

This folder contains sequential and parallel implementations of Probabilistic Roadmap (PRM) motion planning for multiple circular robots moving through a 2D obstacle field. The project includes interactive Pygame simulations and an experiment script for comparing runtime performance.

## Files

- `PRM.py`  
  Sequential PRM implementation. Each robot builds its own roadmap using standard Python loops and Dijkstra search. It includes robot-robot collision detection; when robots collide, all robots stop, rebuild PRMs from their current positions, and continue.

- `pPRM.py`  
  Parallel/vectorized PRM implementation. It uses PyTorch tensor operations for sample filtering, edge checking, and shortest-path relaxation. It can run on CPU, CUDA, or Apple MPS depending on availability.

- `experiments.py`  
  Batch experiment runner. It compares sequential vs parallel CPU timing, then compares parallel CPU vs GPU/MPS timing when an accelerator is available. Results are saved as a CSV and plotted with pandas/matplotlib.

- `experiment_results/`  
  Output directory created by `experiments.py`. It stores timing CSV files and plots.

## Requirements

Python 3.10+ is recommended.

Install the required packages:

```powershell
pip install pygame torch pandas matplotlib
```

If you are using a virtual environment:

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install pygame torch pandas matplotlib
```

For CUDA GPU support, install the PyTorch build that matches your CUDA version from the official PyTorch install instructions.

## Run The Sequential Simulator

```powershell
python .\PRM.py
```

This opens a Pygame window showing:

- gray polygon obstacles
- red robots
- blue PRM paths
- gray path waypoints
- yellow current target waypoints
- green goal region

If two robots collide or are predicted to collide, the sequential simulation stops movement, separates the colliding robots slightly, replans all robots, then continues.

## Run The Parallel Simulator

```powershell
python .\pPRM.py
```

By default, `pPRM.py` chooses the best available PyTorch device in this order:

1. Apple MPS, if available
2. CUDA, if available
3. CPU

You can force a specific device with the `PRM_DEVICE` environment variable:

```powershell
$env:PRM_DEVICE="cpu"
python .\pPRM.py
```

```powershell
$env:PRM_DEVICE="cuda"
python .\pPRM.py
```

```powershell
$env:PRM_DEVICE="mps"
python .\pPRM.py
```

The parallel version uses PyTorch vectorization for PRM computation and a small thread pool for non-blocking replanning during the live simulation.

## Run The Experiments

```powershell
python .\experiments.py
```

The experiment script runs timing comparisons for:

- `5` robots
- `10` robots
- `20` robots
- `50` robots

Experiment parameters:

- robot radius: `10`
- obstacle count: `5`
- PRM samples: `500`
- fixed seed per robot count for fair comparisons
- one additional random-seed run per robot count

The script compares:

- sequential PRM on CPU vs parallel PRM on CPU
- parallel PRM on CPU vs parallel PRM on CUDA/MPS, if available

To skip GPU/MPS timing:

```powershell
python .\experiments.py --skip-gpu
```

To choose a custom output directory:

```powershell
python .\experiments.py --output-dir my_results
```

## Experiment Outputs

By default, results are saved in `experiment_results/`.

Generated files include:

- `prm_timing_results.csv`  
  Raw timing data for each robot count, seed type, implementation, and device.

- `sequential_vs_parallel_cpu.png`  
  Plot comparing sequential CPU time against parallel CPU time.

- `parallel_cpu_vs_gpu.png`  
  Plot comparing parallel CPU time against parallel GPU/MPS time. This is only generated when an accelerator is available.

- `random_seed_run.png`  
  Plot for the additional random-seed experiment.

## Notes About Fairness

The experiment script creates one shared environment per robot count and seed, then runs each implementation on that same obstacle layout, robot positions, and goal. This keeps the sequential and parallel comparisons fair.

The fixed seeds are different for each robot count, but reused across implementations for that robot count. The random-seed run is included as a separate robustness check.

## Common Issues

If Pygame does not open a window, make sure you are running the script from a normal terminal session with desktop access.

If GPU timing is skipped, PyTorch did not detect CUDA or MPS. You can still run the CPU experiments.

If the experiments take a long time, the `50` robot sequential run is usually the slowest. You can reduce `ROBOT_COUNTS` or `N_SAMPLES` in `experiments.py` for faster test runs.

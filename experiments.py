import argparse
import random
import time
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import torch

import PRM as sequential_prm
import pPRM as parallel_prm


ROBOT_COUNTS = [5, 10, 20, 50]
FIXED_SEEDS_BY_ROBOT_COUNT = {
    5: 17341,
    10: 28439,
    20: 39563,
    50: 48611,
}

ROBOT_RADIUS = 10
OBSTACLE_COUNT = 5
N_SAMPLES = 500
GAMMA_PRM = 500.0
D = 2


def configure_modules(robot_count):
    for module in (sequential_prm, parallel_prm):
        module.ROBOT_COUNT = robot_count
        module.ROBOT_RADIUS = ROBOT_RADIUS
        module.OBSTACLE_COUNT = OBSTACLE_COUNT
        module.N_SAMPLES = N_SAMPLES
        module.GAMMA_PRM = GAMMA_PRM
        module.D = D


def make_environment(robot_count, seed):
    configure_modules(robot_count)
    random.seed(seed)
    obstacles = sequential_prm.generate_obstacles()
    robots = sequential_prm.generate_robots(obstacles)
    goal = sequential_prm.generate_goal(obstacles, robots)
    return obstacles, robots, goal


def synchronize_device(device):
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    elif device.type == "mps" and hasattr(torch, "mps"):
        torch.mps.synchronize()


def get_accelerator_device():
    if torch.cuda.is_available():
        return torch.device("cuda")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return None


def time_sequential_prm(obstacles, robots, goal):
    start_time = time.perf_counter()
    paths_found = 0

    for robot in robots:
        prm = sequential_prm.PRM(obstacles, robot.position, goal[0], N_SAMPLES, GAMMA_PRM, D)
        if prm.dijkstra():
            paths_found += 1

    elapsed = time.perf_counter() - start_time
    return elapsed, paths_found


def time_parallel_prm(obstacles, robots, goal, device):
    parallel_prm.PRM_DEVICE = device
    synchronize_device(device)
    start_time = time.perf_counter()
    paths_found = 0

    for robot in robots:
        prm = parallel_prm.PRM(obstacles, robot.position, goal[0], N_SAMPLES, GAMMA_PRM, D)
        if prm.dijkstra():
            paths_found += 1

    synchronize_device(device)
    elapsed = time.perf_counter() - start_time
    return elapsed, paths_found


def run_one_world(robot_count, seed, seed_type, include_gpu):
    obstacles, robots, goal = make_environment(robot_count, seed)
    rows = []

    print(f"Running {seed_type} seed {seed} with {robot_count} robots")

    sequential_time, sequential_paths = time_sequential_prm(obstacles, robots, goal)
    rows.append({
        "seed_type": seed_type,
        "seed": seed,
        "robot_count": robot_count,
        "implementation": "sequential",
        "device": "cpu",
        "time_seconds": sequential_time,
        "paths_found": sequential_paths,
    })

    parallel_cpu_time, parallel_cpu_paths = time_parallel_prm(
        obstacles,
        robots,
        goal,
        torch.device("cpu"),
    )
    rows.append({
        "seed_type": seed_type,
        "seed": seed,
        "robot_count": robot_count,
        "implementation": "parallel",
        "device": "cpu",
        "time_seconds": parallel_cpu_time,
        "paths_found": parallel_cpu_paths,
    })

    accelerator = get_accelerator_device() if include_gpu else None
    if accelerator is not None:
        parallel_gpu_time, parallel_gpu_paths = time_parallel_prm(
            obstacles,
            robots,
            goal,
            accelerator,
        )
        rows.append({
            "seed_type": seed_type,
            "seed": seed,
            "robot_count": robot_count,
            "implementation": "parallel",
            "device": accelerator.type,
            "time_seconds": parallel_gpu_time,
            "paths_found": parallel_gpu_paths,
        })

    return rows


def plot_results(df, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)

    fixed = df[df["seed_type"] == "fixed"].copy()
    fixed["planner"] = fixed["implementation"] + "_" + fixed["device"]

    seq_vs_parallel = fixed[fixed["planner"].isin(["sequential_cpu", "parallel_cpu"])]
    pivot = seq_vs_parallel.pivot(index="robot_count", columns="planner", values="time_seconds")
    ax = pivot.plot(marker="o", title="Sequential PRM vs Parallel PRM on CPU")
    ax.set_xlabel("Robot count")
    ax.set_ylabel("Planning time (seconds)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / "sequential_vs_parallel_cpu.png", dpi=200)
    plt.close()

    parallel_only = fixed[fixed["implementation"] == "parallel"]
    if parallel_only["device"].nunique() > 1:
        pivot = parallel_only.pivot(index="robot_count", columns="device", values="time_seconds")
        ax = pivot.plot(marker="o", title="Parallel PRM CPU vs GPU")
        ax.set_xlabel("Robot count")
        ax.set_ylabel("Planning time (seconds)")
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(output_dir / "parallel_cpu_vs_gpu.png", dpi=200)
        plt.close()

    random_rows = df[df["seed_type"] == "random"].copy()
    if not random_rows.empty:
        random_rows["planner"] = random_rows["implementation"] + "_" + random_rows["device"]
        pivot = random_rows.pivot(index="robot_count", columns="planner", values="time_seconds")
        ax = pivot.plot(marker="o", title="Random Seed Timing Run")
        ax.set_xlabel("Robot count")
        ax.set_ylabel("Planning time (seconds)")
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(output_dir / "random_seed_run.png", dpi=200)
        plt.close()


def parse_args():
    parser = argparse.ArgumentParser(description="Run PRM timing experiments.")
    parser.add_argument("--output-dir", default="experiment_results", help="Directory for CSV and plots.")
    parser.add_argument("--skip-gpu", action="store_true", help="Skip CUDA/MPS parallel PRM timing.")
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    include_gpu = not args.skip_gpu

    if include_gpu and get_accelerator_device() is None:
        print("No CUDA or MPS device found; GPU comparison will be skipped.")

    all_rows = []

    for robot_count in ROBOT_COUNTS:
        fixed_seed = FIXED_SEEDS_BY_ROBOT_COUNT[robot_count]
        all_rows.extend(run_one_world(robot_count, fixed_seed, "fixed", include_gpu))

    random_seeds = {
        robot_count: random.SystemRandom().randint(0, 1_000_000)
        for robot_count in ROBOT_COUNTS
    }
    for robot_count, seed in random_seeds.items():
        all_rows.extend(run_one_world(robot_count, seed, "random", include_gpu))

    df = pd.DataFrame(all_rows)
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "prm_timing_results.csv"
    df.to_csv(csv_path, index=False)
    plot_results(df, output_dir)

    print("\nTiming summary:")
    print(df.to_string(index=False))
    print(f"\nSaved results to {csv_path}")
    print(f"Saved plots to {output_dir}")


if __name__ == "__main__":
    main()

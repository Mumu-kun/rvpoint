#!/usr/bin/env python3
"""
RVPoint gem5 Batch Simulation Runner.
Runs tuples of (file, backend) in parallel/batches using `gem5run`,
pre-building backends and parsing output metrics into JSON.
"""

import argparse
import sys
import os
import time
import subprocess

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEM5RUN_BIN = os.path.join(PROJECT_ROOT, "scripts", "gem5run")

# Default target benchmark tuples: (relative_cpp_file, backend)
DEFAULT_TASKS = [
    ("tests/bench/bench_caravan_radius.cpp", "rvv"),
    ("tests/bench/bench_octree_radius.cpp", "rvv"),
    ("tests/bench/bench_scalar_radius.cpp", "scalar"),
]

def prebuild_backends(backends):
    """Pre-build C++ libraries for requested backends to avoid build locks."""
    print("\033[1m1/2 Pre-building library backends...\033[0m")
    for b in backends:
        print(f"  \033[2mBuilding {b} backend...\033[0m")
        build_script = os.path.join(PROJECT_ROOT, "scripts", "build.sh")
        cmd = [build_script, "--toolchain", "linux", "--backend", b, "--gem5", "--target", "rvpoint"]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            print(f"\033[31m✗ Build failed for {b}:\033[0m\n{res.stderr}")
            sys.exit(1)
    print("  \033[32m✓ Library backends ready.\033[0m\n")

import tempfile

def run_single_job(task):
    cpp_file, backend = task
    abs_cpp = os.path.join(PROJECT_ROOT, cpp_file)
    if not os.path.exists(abs_cpp):
        abs_cpp = cpp_file

    base_name = os.path.splitext(os.path.basename(abs_cpp))[0]
    log_temp = tempfile.NamedTemporaryFile(mode="w+", delete=False)
    cmd = [GEM5RUN_BIN, abs_cpp, "--backend", backend]

    start_t = time.time()
    proc = subprocess.Popen(cmd, stdout=log_temp, stderr=subprocess.STDOUT, cwd=PROJECT_ROOT)
    
    return {
        "task": task,
        "proc": proc,
        "base_name": base_name,
        "backend": backend,
        "log_file": log_temp.name,
        "log_handle": log_temp,
        "start_time": start_t
    }

def main():
    parser = argparse.ArgumentParser(description="Run batches of gem5 simulations for (file, backend) tuples.")
    parser.add_argument("-j", "--jobs", type=int, default=4, help="Maximum parallel gem5 jobs (default: 4)")
    parser.add_argument("--tuple", action="append", nargs=2, metavar=("FILE", "BACKEND"),
                        help="Custom (file, backend) tuple, e.g. --tuple tests/bench/bench_caravan_radius.cpp rvv")
    args = parser.parse_args()

    if args.tuple:
        tasks = [(t[0], t[1]) for t in args.tuple]
    else:
        tasks = DEFAULT_TASKS

    backends_to_build = sorted(list({t[1] for t in tasks}))
    prebuild_backends(backends_to_build)

    total = len(tasks)
    print(f"\033[1m2/2 Running {total} gem5 simulation jobs (max parallel: {args.jobs})...\033[0m")

    start_all = time.time()
    active_jobs = []
    task_queue = list(tasks)
    completed_count = 0

    while task_queue or active_jobs:
        # Fill up to max parallel jobs
        while len(active_jobs) < args.jobs and task_queue:
            t = task_queue.pop(0)
            job_info = run_single_job(t)
            print(f"  \033[2m▶ Launching:\033[0m {job_info['base_name']} ({job_info['backend']})")
            active_jobs.append(job_info)

        # Monitor active jobs
        time.sleep(0.05)
        still_active = []
        for job in active_jobs:
            poll = job["proc"].poll()
            if poll is None:
                still_active.append(job)
            else:
                job["log_handle"].close()
                if os.path.exists(job["log_file"]):
                    try:
                        os.remove(job["log_file"])
                    except OSError:
                        pass
                completed_count += 1

        active_jobs = still_active
        elapsed = time.time() - start_all
        mins = int(elapsed // 60)
        secs = elapsed % 60
        print(f"\r  \033[36m⏳ Progress: [{completed_count}/{total} completed] | Active jobs: {len(active_jobs)}\033[0m \033[1m{mins:02d}:{secs:05.2f}\033[0m", end="", flush=True)

    total_wall = time.time() - start_all
    mins = int(total_wall // 60)
    secs = total_wall % 60
    print(f"\r  \033[32m\033[1m✓ All {total} benchmark jobs completed.\033[0m \033[2m(wall: {mins:02d}:{secs:05.2f}s)\033[0m\n")

    out_dir = os.path.join(PROJECT_ROOT, "output", "gem5")
    print("\033[1mGenerated JSON Metric Files in ./output/gem5/:\033[0m")
    if os.path.exists(out_dir):
        json_files = sorted([f for f in os.listdir(out_dir) if f.endswith(".json")])
        for jf in json_files:
            full_p = os.path.join(out_dir, jf)
            sz = os.path.getsize(full_p)
            print(f"  \033[36m{jf}\033[0m ({sz} bytes)")
    print()

if __name__ == "__main__":
    main()

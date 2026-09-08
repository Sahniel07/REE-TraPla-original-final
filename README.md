# Trajectory Planning Accuracy with Posit and IEEE-754 Arithmetic

Code accompanying the bachelor's thesis *Evaluation of Trajectory Planning
Accuracy with Posit and IEEE-754 Floating-Point Arithmetic* (Sahniel Satyan,
Technische Hochschule Ingolstadt, Research Institute AImotion Bavaria).

The planner is the game-theoretic trajectory planner of Lucente et al.
(DeepGame-TP), which solves a Generalized Nash Equilibrium problem over the
vehicles in a scene. This repository makes the arithmetic type it computes in a
build option, so the same algorithm can be run in IEEE 754 binary16, binary32
and binary64 and in posits of 16 and 32 bits at several exponent sizes, and the
resulting trajectories compared.

## 1. Setup

Linux only: `sweep.sh` is a bash script and the build uses the GNU toolchain.
On Windows use WSL (`wsl --install`, then reboot) and work in the Linux shell.

```bash
sudo apt update
sudo apt install build-essential cmake cmake-curses-gui libeigen3-dev python3 python3-pip
pip3 install -r requirements.txt
```

`pip3` comes from the `apt` line, so run these as two steps and check both
before going on:

```bash
cmake --version
python3 -c "import pandas, matplotlib, numpy; print('python deps ok')"
```

A missing compiler stops the build with a clear message. Missing Python
packages do not: the planner still writes its CSVs, fails to draw the figure,
and reports only a warning.

The posit and half-precision types come from the Stillwater Universal library,
which is header-only and not vendored here:

```bash
mkdir -p thirdparty
git clone https://github.com/stillwater-sc/universal.git thirdparty/universal
git -C thirdparty/universal checkout 998f5aa4e3e046eb4f5d1ea3fb81073bc1cb223e
```

That commit is release `v4.8.9-7-g998f5aa4e`, the version the reported results
were produced with. Other versions have not been tested here.

## 2. Run the sweep

Edit the `SETTINGS` block at the top of `sweep.sh`. Three choices, nothing else:

```bash
SCENARIO="intersection"     # intersection | crossing | merge
TRIG_PATHS=(poly)           # poly, upscale, or both
OUTDIR="saved/Intersection" # relative to this script
```

| Setting | Meaning |
|---|---|
| `SCENARIO` | `intersection` (eight vehicles at a crossroads, the reference), `crossing` (four at an unsignalled junction), `merge` (one joining a through lane from a ramp) |
| `TRIG_PATHS` | `poly` evaluates sin/cos/tan natively in the format under study from the fdlibm minimax polynomial, and is the path the reported results use. `upscale` converts to fp64, calls the C library, and rounds back. Listing both runs the sweep once per path. |
| `OUTDIR` | where the CSVs are collected; the figures go to `media/<basename>/` |

```bash
./sweep.sh --dry-run   # list the configurations, build nothing
./sweep.sh             # build and run them
```

Each of the twelve formats runs with nondimensionalization off and on, so the
default is 24 runs, each built from scratch. A sweep whose `OUTDIR` already
holds results moves that directory aside to `<OUTDIR>.superseded-<timestamp>`,
keeping runs from different code states out of one report.

## 3. Build a single configuration

Following the original REE-TraPla, the options are edited interactively:

```bash
mkdir build && cd build
cmake ..
ccmake .              # Enter changes a value, c configures, g generates
cmake --build .
./dynamic_game_trajectory_planner
```

`ccmake` comes from `cmake-curses-gui`. The same options can be passed with
`-D`, which is what `sweep.sh` does:

```bash
cmake -S . -B build -DENABLE_POSIT=ON -DREAL_BITS=16 -DPOSIT_ES=2 \
      -DENABLE_QUANTIZATION=ON -DTRIG_PATH=poly -DSCENARIO=intersection
```

| Option | Values | Meaning |
|---|---|---|
| `ENABLE_POSIT` | `ON`, `OFF` | posit, or the IEEE 754 type of that width |
| `REAL_BITS` | `16`, `32`, `64` | width of the working format |
| `POSIT_ES` | `0`–`4` | posit exponent size (posit only) |
| `ENABLE_QUANTIZATION` | `ON`, `OFF` | nondimensionalization |
| `TRIG_PATH` | `poly`, `upscale` | trigonometric path |
| `SCENARIO` | `intersection`, `crossing`, `merge` | scene to plan |

Run the binary from its build directory: it writes its CSVs relative to the
working directory and invokes `../plot_run.py` to draw the figure.

## 4. Output

| Where | What |
|---|---|
| `trajectories_*.csv` | the planned trajectory of every vehicle |
| `lanes_*.csv` | the centre lane each vehicle follows |
| `media/**/*.png` | the trajectory plot of each run |
| `cv_report.tsv` | sweep only: one row per run, constraint violation by family and convergence state |
| `log_*.stdout`, `log_*.stderr` | sweep only: the raw output of each run |

A sweep collects these into `$OUTDIR`; a single run leaves them in its build
directory. Files are named `<scenario>_<format>_<quantization>_<trig>`, so no
run overwrites another.

A single run also prints two lines that are not written to any file, so
redirect stdout to keep them:

```
CV overall=No input_limits=No collision_avoidance=No lane_boundaries=No
CONV iterations=100 iter_limit=100 capped=yes termination=iteration_limit ...
```

`CV` is the constraint violation split by family. `CONV` says whether the run
met the stationarity test or stopped at the iteration limit; a capped run is
the planner's output after a fixed budget, not a solved equilibrium.

`saved/` and `media/` do not exist in a fresh checkout. Both are created on the
first run, so the results in them are always ones you generated yourself.

## 5. Metrics

Neither a sweep nor a single run computes the reported metrics. Both produce
trajectories; `analyze_scenarios.py` turns them into numbers, and reads any
directory holding trajectory CSVs:

```bash
python3 analyze_scenarios.py saved/Intersection   # a sweep's output
python3 analyze_scenarios.py build                # a single run
```

```
format             cost l   l/l_fp64    min_sep       path  c_v
posit<16,2>         84.52     0.9715      2.576      93.10  no
fp64                87.00     1.0000      2.500      91.09  no
```

It reports the accumulated cost `l`, the ratio `l/l_fp64`, the minimum
separation any pair reaches, the path length, and whether the safety distance
was violated. A run whose trajectory holds a non-finite value is reported as
such rather than scored, since a comparison against NaN passes the collision
test silently. A run that produced no motion is reported as stalled.

`l/l_fp64` needs an fp64 run in the same directory, since the ratio is taken
against fp64 there. A sweep always has one. For a single run, build and run the
same scenario at `REAL_BITS=64` into the same build directory to fill it in.

`make_tables.py` emits the same numbers as LaTeX table bodies, reading
`saved/Intersection`, `saved/Crossing` and `saved/Merge`:

```bash
python3 make_tables.py
```

Both scripts use the standard library only.

If the figures are missing but the CSVs are there, the plotting dependencies
were absent when the run happened. Install them, then draw the figures from the
stored trajectories without running the planner again:

```bash
for f in saved/Intersection/trajectories_*.csv; do python3 plot_run.py "$f"; done
```

## 6. Layout

```
sweep.sh              the entry point; produces every reported result
analyze_scenarios.py  tabulates cost, separation and path length per format
make_tables.py        emits those numbers as LaTeX table bodies
plot_run.py           draws one run's trajectory figure, called by the planner
CMakeLists.txt        build configuration, exposing the options above
include/              planner headers, scenario definitions, trig paths
src/                  planner implementation
```

## References

The planner this repository modifies:

```bibtex
@article{deepgametp,
  author  = {Lucente, Giovanni and Maarssoe, Mikkel Skov and Konthala, Sanath Himasekhar
             and Abulehia, Anas and Dariani, Reza and Schindler, Julian},
  title   = {{DeepGame-TP}: Integrating Dynamic Game Theory and Deep Learning for Trajectory Planning},
  journal = {IEEE Open Journal of Intelligent Transportation Systems},
  volume  = {5},
  pages   = {873--888},
  year    = {2024},
  doi     = {10.1109/OJITS.2024.3515270}
}
```

The vectorization and nondimensionalization work this codebase derives from:

```bibtex
@INPROCEEDINGS{REE_TraPla_Xu_2026,
  author={Xu, Wenguang and Lucente, Giovanni and Membarth, Richard},
  booktitle={2026 IEEE Intelligent Vehicles Symposium (IV)},
  title={Energy- and Runtime-Efficient Trajectory Planning via SIMD Vectorization},
  year={2026},
  organization={IEEE}
}
```

The arithmetic library providing the posit and half-precision types:

```bibtex
@article{omtzigt2023,
  author  = {Omtzigt, E. Theodore L. and Quinlan, James},
  title   = {Universal Numbers Library: Multi-format Variable Precision Arithmetic Library},
  journal = {Journal of Open Source Software},
  volume  = {8},
  number  = {83},
  pages   = {5072},
  year    = {2023},
  doi     = {10.21105/joss.05072}
}
```

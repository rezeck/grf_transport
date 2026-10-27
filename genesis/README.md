# GRF object transport in Genesis (no ROS, no Gazebo)

Branch `genesis-sim`. Ten HeRo robots transport a box to a target using the
Gibbs Random Field controller from the paper
[Cooperative Object Transportation using Gibbs Random Fields](https://arxiv.org/abs/2109.13734)
(Rezeck, Assunção, Chaimowicz, IROS 2021). The controller is a port of
`src/grf_colletive_transport.cpp` to Python (`genesis/grf.py`) and runs in the
[Genesis](https://genesis-world.readthedocs.io/en/latest/) physics engine.
Each robot only uses its 0.5 m laser: no communication, no leader.

## Requirements

- Python **3.12** (the pinned wheels were tested on 3.12)
- `git`
- macOS or Linux. The demo runs on CPU; no GPU is needed.
- About 3 GB of disk for PyTorch and Genesis.

## Setup

The robot and arena models live in a sibling repository, `hero_common`. Clone
both repositories **side by side**:

```bash
mkdir grf-workspace && cd grf-workspace

git clone -b genesis-sim https://github.com/rezeck/grf_transport.git
git clone https://github.com/verlab/hero_common.git

cd grf_transport
python3.12 -m venv .venv-genesis
.venv-genesis/bin/pip install -r genesis/requirements.txt
```

Expected layout:

```
grf-workspace/
├── grf_transport/   # this repository, branch genesis-sim
└── hero_common/     # robot URDFs, meshes and the arena
```

If `hero_common` lives elsewhere, point to it with an environment variable:

```bash
export HERO_COMMON=/path/to/hero_common
```

On macOS, cloning `hero_common` prints a warning about two paths that differ
only by case (`MT3608.lbr` / `mt3608.lbr`). It is a PCB library file that the
simulation does not use, so you can ignore it.

## Run

Headless (no window, prints a summary at the end):

```bash
.venv-genesis/bin/python genesis/run_demo.py --headless
```

With the viewer (omit `--headless`):

```bash
.venv-genesis/bin/python genesis/run_demo.py
```

Options:

| Flag | Default | Meaning |
|------|---------|---------|
| `--headless` | off | Do not open the viewer |
| `--steps N` | 30000 | Simulation steps after settling (dt = 0.01 s, so 30000 steps = 300 s) |
| `--steps-settle N` | 200 | Steps to let the robots and the box settle before the controller starts |

Quick smoke test (about 1 minute, the box will not reach the target):

```bash
.venv-genesis/bin/python genesis/run_demo.py --headless --steps 600
```

## What to expect

- The scene has a 4 x 4 m arena, ten HeRo robots, a 0.4 m box at
  `(0, 1.1)` and a target at `(0, -1.2)`.
- The robots start away from the box and wander until their laser sees it, which
  takes about 2 simulated minutes. Then they circle it and push it to the target.
- The controller runs at 10 Hz. On a laptop CPU the simulation is slower than
  real time, and the 600-step smoke test takes about a minute (mostly start-up).
- The process exits with code `0` when the box ends within 0.10 m of the target
  and with code `1` otherwise (it prints `transporte incompleto`). With the
  default parameters the box gets to about 0.2 m of the target and oscillates
  around it, so a full 300 s run currently exits with `1`. This is a known issue
  with the stopping behaviour, not an installation problem.

## Files

| File | Content |
|------|---------|
| `genesis/grf.py` | GRF controller: potentials, Metropolis-Hastings sampling, laser model |
| `genesis/run_demo.py` | Scene construction, contact parameters, differential drive, main loop |
| `genesis/requirements.txt` | Pinned `torch` and `genesis-world` versions |
| `config/params.yaml` | Original parameters, copied as constants into `grf.py` |
| `models/rectangular_prism/` | Box and target URDFs |

`GENESIS.md` at the repository root is a shorter description of the same branch.

## Troubleshooting

- **`hero_common não encontrado`**: the clone is not next to `grf_transport`;
  clone it there or set `HERO_COMMON`.
- **`No matching distribution found for torch==2.14.1`**: your Python is not
  3.12, or `pip` is outdated. Recreate the venv with `python3.12` and run
  `pip install --upgrade pip`.
- **Robots fall through the floor or the box lifts**: the script aborts with a
  message. This indicates a different `genesis-world` version; keep the pins in
  `requirements.txt`.

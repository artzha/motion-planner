# motion-planner

A ROS-free C++ kinodynamic differential-drive **hybrid A\*** planner with
optional Python bindings.

## Installation

```bash
# system deps + submodules
sudo apt-get install -y libgoogle-glog-dev libgflags-dev libeigen3-dev liblua5.2-dev
git submodule update --init --recursive

# python deps (for the bindings + demo)
pip install pybind11 numpy matplotlib

# build (produces ./build/navigation and ./build/hybrid_astar*.so)
make
```

Make the `hybrid_astar` module importable from anywhere by copying it into your
interpreter's `site-packages`:

```bash
cp build/hybrid_astar*.so \
  "$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
```

## Demo

C++ obstacle-avoidance test (writes `astar_path.csv`):

```bash
./build/navigation --test_avoidance --nav_config config/navigation.lua
```

Python demo: plans to a fan of goals around obstacles and writes `plan.png`:

```bash
python scripts/plot_plan.py
```

![Example hybrid A* plan](plan.png)

## Additional details

- Run both demos from the repo root so `config/navigation.lua` resolves.
- If `make` builds the module against the wrong interpreter, reconfigure with
  `cmake -S . -B build -DPython3_EXECUTABLE=$(which python)` and rebuild.
- `plan()` takes an `(N, 2)` base_link point cloud (x forward, y left) and a
  `goal=(x, y)`, and returns an `(N, 5)` array of `x, y, theta, v, omega`.

### Layout

```
config/navigation.lua        # planner + grid parameters (loaded at runtime)
scripts/plot_plan.py         # Python demo
src/navigation/
  astar.h                    # generic A* search
  domain.h                   # differential-drive kinodynamic domain
  wavefront.{h,cc}           # obstacle-aware heuristic
  navigation_config.{h,cc}   # lua config loader
  navigation_main.cc         # C++ demo (--test_avoidance)
  python_bindings.cc         # pybind11 module `hybrid_astar`
src/{config_reader,shared}/  # AMRL submodules
```

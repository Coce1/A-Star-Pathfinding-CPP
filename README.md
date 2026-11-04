# A* Pathfinding & Motion Planning in C++

A modern, high-performance C++ implementation of the **A* (A-Star) search algorithm** tailored for 2D grid-based robot motion planning, obstacle avoidance, and optimal trajectory generation.

---

## 📌 Features

- **Optimal & Complete Pathfinding:** Guarantees the shortest path on a 2D discrete grid (4-connected) using the Manhattan heuristic.
- **Dynamic Grid & Obstacle Mapping:** Flexible cost evaluation with real-time obstacle checking, plus a safety margin (inflated obstacles) around walls.
- **Optimized Data Structures:** Priority-queue open list and closed flags for fast node exploration and low memory overhead, with tie-breaking on the heuristic.
- **Path Smoothing & Replanning:** Straight-line shortcuts checked on the grid, and click-to-replan from the current estimated position.
- **State Estimation:** The robot is driven from an **Extended Kalman Filter** (noisy GPS + noisy odometry), not from the ground truth.
- **Graphical Visualization:** Integrated real-time rendering using **Raylib**.
- **Vector & Matrix Math:** Utilizes **Eigen** for robust geometric calculations and transformations.

---

## 🧠 Mathematical Foundations

The A* algorithm evaluates nodes based on the standard cost function:

$$f(n) = g(n) + h(n)$$

Where:
- $g(n)$: The exact cost to reach node $n$ from the starting position.
- $h(n)$: The admissible heuristic estimate of the cost from node $n$ to the goal (Manhattan Distance, suited to a 4-connected grid):

$$h(n) = \lvert x_{\mathrm{goal}} - x_n \rvert + \lvert y_{\mathrm{goal}} - y_n \rvert$$

- $f(n)$: The total estimated cost of the cheapest solution path through node $n$.

---

## 🛠️ Tech Stack & Dependencies

- **Language:** C++17 / C++20
- **Rendering Engine:** [Raylib](https://www.raylib.com/) (installed via NuGet / vcpkg)
- **Math Library:** [Eigen 3](https://eigen.tuxfamily.org/)
- **IDE / Toolchain:** Microsoft Visual Studio (MSVC x64), or CMake

---

## 🚀 Getting Started

### Prerequisites
- Visual Studio 2026 (toolset `v145`, with *Desktop development with C++* workload)
- NuGet Package Manager
- Eigen 3, with the environment variable `EIGEN3_INCLUDE_DIR` pointing to the folder that contains `Eigen/`

### Installation & Build
1. Clone the repository:
   ```bash
   git clone https://github.com/Coce1/A-Star-Pathfinding-CPP.git
   ```
2. Set `EIGEN3_INCLUDE_DIR` (e.g. `C:\libs\eigen-3.4.1`) and restart Visual Studio.
3. Open `Planification et Asservissement (A + EKF).slnx`, restore the NuGet packages, select `Release | x64` and press **F5**.

Alternatively, with CMake (Windows / Linux):
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Controls
- **Left click:** set a new goal (replans from the estimated position)
- **R:** reset the simulation
- Red: GPS | Blue: EKF estimate | Green: ground truth | Orange: planned path | Light red: safety margin

---
## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

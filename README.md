# A* Pathfinding & Motion Planning in C++

A modern, high-performance C++ implementation of the **A* (A-Star) search algorithm** tailored for 2D grid-based robot motion planning, obstacle avoidance, and optimal trajectory generation.

---

## 📌 Features

- **Optimal & Complete Pathfinding:** Guarantees the shortest path on a 2D discrete grid using Euclidean heuristic distance.
- **Dynamic Grid & Obstacle Mapping:** Flexible cost evaluation with real-time obstacle checking.
- **Optimized Data Structures:** Efficient open/closed list management for fast node exploration and low memory overhead.
- **Graphical Visualization:** Integrated real-time rendering using **Raylib**.
- **Vector & Matrix Math:** Utilizes **Eigen** for robust geometric calculations and transformations.

---

## 🧠 Mathematical Foundations

The A* algorithm evaluates nodes based on the standard cost function:

$$f(n) = g(n) + h(n)$$

Where:
- $g(n)$: The exact cost to reach node $n$ from the starting position.
- $h(n)$: The admissible heuristic estimate of the cost from node $n$ to the goal (Euclidean Distance):

$$h(n) = \sqrt{(x_{\text{goal}} - x_n)^2 + (y_{\text{goal}} - y_n)^2}$$

- $f(n)$: The total estimated cost of the cheapest solution path through node $n$.

---

## 🛠️ Tech Stack & Dependencies

- **Language:** C++17 / C++20
- **Rendering Engine:** [Raylib](https://www.raylib.com/) (installed via NuGet / vcpkg)
- **Math Library:** [Eigen 3](https://eigen.tuxfamily.org/)
- **IDE / Toolchain:** Microsoft Visual Studio (MSVC x64)

---

## 🚀 Getting Started

### Prerequisites
- Visual Studio 2022 (with *Desktop development with C++* workload)
- NuGet Package Manager

### Installation & Build
1. Clone the repository:
   ```bash
   git clone [https://github.com/Coce1/A-Star-Pathfinding-CPP.git](https://github.com/Coce1/A-Star-Pathfinding-CPP.git)

---

## 📂 Project Structure

```text
├── .gitignore                                    # Visual Studio and build artifact exclusions
├── packages.config                               # NuGet dependency definitions (Raylib)
├── Planification et Asservissement (A + EKF).slnx # Visual Studio Solution configuration
└── main.cpp                                      # Grid engine, A* algorithm, and Raylib rendering loop
```

## 📄 License

`This project is licensed under the MIT License - see the LICENSE file for details.` : Le paragraphe descriptif de la licence.

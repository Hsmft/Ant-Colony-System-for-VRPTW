# Ant Colony System for the Vehicle Routing Problem with Time Windows (VRPTW)

![Language](https://img.shields.io/badge/language-C%2B%2B-blue.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository contains a C++ implementation of an **Ant Colony System (ACS)**, a popular variant of Ant Colony System (ACS), to solve the **Vehicle Routing Problem with Time Windows (VRPTW)**. The algorithm aims to find near-optimal solutions by minimizing the number of vehicles and the total travel distance.

---

## 📋 Project Overview

This solver uses the Ant Colony System metaheuristic, which is inspired by the foraging behavior of real ants. Artificial ants iteratively construct solutions, communicating indirectly through artificial pheromone trails.

The solver implements the following workflow:

1.  **Pheromone & Heuristic Initialization:** The pheromone matrix is initialized based on a greedy nearest-neighbor solution. The heuristic information matrix is based on the inverse distance between customers.
2.  **Probabilistic Solution Construction:** Each ant in the colony constructs a feasible set of routes. At each step, an ant selects the next customer to visit based on a probabilistic rule that balances:
    * The **pheromone level** on the path (favoring paths chosen by other successful ants).
    * The **heuristic information** (favoring shorter paths).
3.  **Pheromone Update Rules:**
    * **Local Pheromone Update:** As an ant traverses an arc, the pheromone level on that arc is immediately reduced. This encourages subsequent ants in the same iteration to explore different paths, increasing diversification.
    * **Global Pheromone Update:** After all ants have completed their routes in an iteration, the pheromone on the arcs belonging to the best solution found in that iteration is increased. This reinforces good pathways for the next generation of ants.
4.  **Local Search (Optional):** The framework includes an optional local search phase that can be applied to the solutions found by the ants to further refine them and find local optima.

---

## 🛠️ Technologies Used

* **Language:** C++ (utilizing C++11 features like `<chrono>` and `<random>`)
* **Libraries:** C++ Standard Library only. No external optimization libraries were used.

---

## 🚀 How to Compile and Run

### Compilation
You can compile the source code using a standard C++ compiler like g++.

```bash
g++ -std=c++11 -o solver hw5.cpp
```

### Execution
The program is run from the command line with the following arguments:

```bash
./solver [instance-file-path] [max-execution-time] [max-evaluations]
```
* `instance-file-path`: The path to the problem instance file (e.g., `instances/C101.txt`).
* `max-execution-time`: The maximum run time in seconds. Use `0` for no time limit.
* `max-evaluations`: The maximum number of objective function evaluations. Use `0` for no limit.

**Example:**
```bash
./solver instances/C101.txt 60 0
```
This command runs the solver on the `C101.txt` instance for a maximum of 60 seconds.

---

## 📄 License
This project is licensed under the MIT License.
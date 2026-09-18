# Tower of Hanoi State-Space Search Report

**Course**: Foundations of Artificial Intelligence (FAI IA 1)  
**Task**: Question 13 - State-Space Search Model

---

## 1. Problem Formulation
The Tower of Hanoi puzzle is modeled as an explicit State-Space Search Problem:
- **State Space**: Encoded as a 3-tuple `(Peg A, Peg B, Peg C)`. Total reachable state count $S = 3^N$.
- **Initial State**: `((3, 2, 1), (), ())`
- **Goal State**: `((), (), (3, 2, 1))`
- **Transitions**: Moving a top disk from one peg to another without placing a larger disk on a smaller one.

---

## 2. Experimental Benchmark Results

| Disks ($N$) | Algorithm | Solution Moves | Nodes Expanded | Time (s) |
| :---: | :---: | :---: | :---: | :---: |
| 3 | BFS | 7 | 27 | < 0.001 |
| 3 | A* | 7 | 12 | < 0.001 |
| 4 | BFS | 15 | 81 | < 0.002 |
| 4 | A* | 15 | 35 | < 0.001 |
| 5 | BFS | 31 | 243 | ~0.008 |
| 5 | A* | 31 | 98 | ~0.003 |

---

## 3. Analysis
1. **Optimality**: Both BFS and A* find the optimal path length ($2^N - 1$).
2. **Efficiency**: A* with disk displacement heuristic expands significantly fewer states than BFS as $N$ scales.
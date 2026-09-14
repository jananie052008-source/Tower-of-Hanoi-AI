# Tower of Hanoi - State Space Search Model

This repository models the classic **Tower of Hanoi** puzzle as an explicit **State-Space Search Problem** for the *Foundations of Artificial Intelligence* assignment.

## 1. State Space Mapping

- **State Representation**: Tuple of 3 tuples `(Peg_A, Peg_B, Peg_C)` where numbers represent disk sizes (1 = smallest, N = largest).
- **Start State (N=3)**: `((3, 2, 1), (), ())`
- **Goal State (N=3)**: `((), (), (3, 2, 1))`
- **State Complexity**: For N disks on 3 pegs, total reachable states $S = 3^N$. For N=3, total states = 27.

## 2. Implemented Algorithms

1. **Breadth-First Search (BFS)**: Uninformed search exploring graph levels to guarantee optimal sequence ($2^N - 1$ moves).
2. **A* Search**: Informed search using an admissible heuristic (count of disks remaining off target Peg C).

## 3. How to Run

Execute the script from the root folder:

```bash
python src/hanoi_search.py
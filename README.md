# Project 13: Tower of Hanoi Search Model

**Course**: Foundations of Artificial Intelligence (FAI IA 1)

## Project Overview
This project models the classic **Tower of Hanoi** puzzle as an explicit **State-Space Search Problem** using BFS and A* Search algorithms instead of direct recursion.

---

## 1. Mapping Puzzle to State-Space Search

| Search Concept | Puzzle Mapping |
| :--- | :--- |
| **State Space** | All valid peg arrangements $(A, B, C)$ where disk order is preserved (larger under smaller). Total states = $3^N = 27$ for 3 disks. |
| **Initial State** | `((3, 2, 1), (), ())` — All disks on Peg A. |
| **Goal State** | `((), (), (3, 2, 1))` — All disks moved to Peg C. |
| **Actions / Transitions** | Move the top disk of one peg to another peg, provided it lands on a larger disk or an empty peg. |
| **Path Cost** | $g(n) = 1$ per move (unweighted unit action cost). |

---

## 2. Requirements Implementation

- **State Representation**: Enforced via tuples `(Peg_A, Peg_B, Peg_C)` for immutability and efficient hash lookup.
- **Search Return**: Returns both the optimal step-by-step move sequence and intermediate state configurations.
- **Algorithms**:
  - **BFS (Uninformed)**: Guarantees optimal path length ($2^N - 1 = 7$ moves for 3 disks).
  - **A* Search (Informed)**: Uses heuristic $h(n) = N - \text{disks on Peg C}$ to guide search efficiently.

---

## 3. How to Run & Verify

Run the Python script from the project root:

```bash
python src/hanoi_search.py
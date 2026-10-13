# Tower of Hanoi Search Model – Robotic Container-Stacking Planner

## Project Overview

This project explores Artificial Intelligence (AI) search algorithms using the Tower of Hanoi problem. It represents the problem as a state space and finds a sequence of legal moves from an initial state to a goal state.

The project demonstrates how search techniques can be applied to planning problems related to robotic container stacking, where objects must be moved while following placement constraints.

## Objectives

- Represent the Tower of Hanoi problem using states and legal transitions.
- Implement Breadth-First Search (BFS) to find a solution.
- Implement A* search using a heuristic to guide the search.
- Display the solution as step-by-step state transitions.
- Compare the algorithms based on solution length, expanded states, and execution time.
- Explore the relationship between search algorithms and robotic planning.

## Technologies Used

- **Programming language:** Python
- **Libraries:** `heapq`, `collections`, `time`
- **Development environment:** Visual Studio Code
- **Version control:** Git and GitHub

The current search implementation uses Python's standard library and does not require additional packages.

## Algorithms

### 1. Breadth-First Search (BFS)

BFS explores states level by level using a queue. For a problem where each move has equal cost, it finds a solution with the minimum number of moves.

### 2. A* Search

A* uses the evaluation function:

`f(n) = g(n) + h(n)`

Where:
- `g(n)` is the cost of reaching the current state.
- `h(n)` is the heuristic estimate of the remaining cost.
- `f(n)` estimates the total cost of a solution through that state.

The current heuristic estimates how many disks are not yet on the target peg. This guides the search toward the goal.

## State Representation

The problem is represented using three pegs: A, B, and C. Each peg is stored as a tuple of disks, with the top disk at the end.

For three disks:

- **Initial state:** All disks are on Peg A.
- **Goal state:** All disks are on Peg C.
- **Legal move:** Move one top disk to an empty peg or onto a larger disk.

The program generates legal successor states and records the move used to reach each state.

## Program Features

- Configurable number of disks.
- Legal state-transition generation.
- BFS and A* search implementations.
- Step-by-step solution display.
- Comparison of solution moves and expanded states.
- Execution-time benchmark for three, four, and five disks.

## How to Run

### Prerequisites

Install Python 3 and Git, and open the project folder in Visual Studio Code.

### Run the main program

From the project root directory, run:

```bash
python src/main.py
```

The program should display the algorithm comparison, the A* solution path, and the performance benchmark if the implementation executes successfully.

### Run the tests

If `pytest` is installed, run:

```bash
python -m pytest
```

Review the test output to determine whether all tests pass.

## Applications

Search and planning techniques like those demonstrated here are relevant to:

- Robotic object manipulation.
- Warehouse and container organization.
- Automated movement planning.
- State-space problem solving.
- Decision-making under movement constraints.

The Tower of Hanoi problem is a simplified model for learning these concepts. A real robotic container-stacking system would require additional constraints, such as container weight, stability, physical dimensions, collision avoidance, and robot movement limitations.

## Limitations and Future Enhancements

- Extend the model to include container weights and stacking constraints.
- Add a Constraint Satisfaction Problem (CSP) solver for container placement.
- Develop a Blocks World planner with explicit actions, preconditions, and effects.
- Add visualisation of peg states and movement sequences.
- Compare search algorithms on larger problem instances.
- Improve error handling and expand automated tests.

These enhancements should be documented as completed only after they have been implemented and tested.

## Conclusion

This project demonstrates how BFS and A* can search a Tower of Hanoi state space to find legal move sequences. Comparing the algorithms helps illustrate the role of uninformed and heuristic-guided search in Artificial Intelligence. The same fundamental ideas provide a starting point for more advanced robotic planning and container-stacking applications.

## Repository

GitHub: https://github.com/jananie052008-source/Tower-of-Hanoi-AI

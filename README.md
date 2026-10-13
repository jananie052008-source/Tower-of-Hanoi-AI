# Tower of Hanoi Search Model – Robotic Container-Stacking Planner

## Project Overview

This project demonstrates Artificial Intelligence search and planning techniques using the Tower of Hanoi problem and a robotic container-stacking scenario.

It explores state-space search, heuristic search, planning actions, Blocks World, and Constraint Satisfaction Problems (CSPs). These concepts help demonstrate how an intelligent system can find sequences of actions and organize objects subject to constraints.

## Objectives

- Represent Tower of Hanoi as a state-space search problem.
- Implement Breadth-First Search (BFS), A* search, and Depth-First Search (DFS).
- Generate legal moves and display solution paths.
- Compare search methods using solution length and explored nodes.
- Demonstrate STRIPS-style planning and Blocks World.
- Explore CSP-based container-stack assignment.
- Model robotic actions using preconditions and effects.

## Technologies Used

- **Language:** Python 3
- **Libraries:** Python standard library, including `heapq`, `collections`, and `time`
- **Development environment:** Visual Studio Code
- **Version control:** Git and GitHub
- **Testing:** pytest

## Algorithms and AI Techniques

### 1. Breadth-First Search (BFS)

BFS explores states level by level using a queue. When every move has equal cost, BFS finds a solution with the minimum number of moves.

### 2. A* Search

A* uses the evaluation function:

`f(n) = g(n) + h(n)`

- `g(n)` is the cost of reaching the current state.
- `h(n)` estimates the remaining cost to reach the goal.
- `f(n)` estimates the total cost through the current state.

The implementation uses a heuristic based on the number of disks not yet on the target peg. Its effectiveness and optimality should be evaluated against the implementation and test results.

### 3. Depth-First Search (DFS)

DFS explores a path deeply before backtracking. It can find a solution, but it does not generally guarantee the shortest solution.

### 4. STRIPS-Style Planning

The planner represents actions using preconditions and effects. An action can be performed when its preconditions hold, and its effects update the current state.

### 5. Blocks World

The Blocks World example demonstrates planning how to rearrange blocks from an initial arrangement into a target arrangement.

### 6. Constraint Satisfaction Problem (CSP)

The container-stacking example assigns containers to stacks while considering the constraints implemented by the solver. The exact enforced constraints should be verified in the source code before being claimed in detail.

## State Representation

The Tower of Hanoi model uses three pegs: A, B, and C. Each peg is represented as a tuple of disks, with the top disk at the end.

- **Initial state:** All disks are on Peg A.
- **Goal state:** All disks are on Peg C.
- **Legal move:** Move the top disk to an empty peg or onto a larger disk.

The program generates legal successor states and uses search algorithms to find paths to the goal.

## Program Features

- Configurable Tower of Hanoi disk count.
- Legal state-transition generation.
- BFS, A*, and DFS search implementations.
- Step-by-step movement output.
- Comparison of solution lengths and explored nodes.
- Performance examples for different problem sizes.
- STRIPS-style planning.
- Blocks World planning.
- CSP-based container-stack assignment.
- Robotic container actions with preconditions and effects.
- Goal-state verification for the robotic plan.

## How to Run

From the project root directory, run:

```bash
python src/main.py
```

To run the automated tests:

```bash
python -m pytest
```

The current verified test run reported **3 passed**. Run the tests again after any code changes.

## Applications

The techniques demonstrated in this project can help explain:

- Robotic object manipulation.
- Warehouse and container organization.
- Automated movement and action planning.
- Constraint-based resource arrangement.
- State-space search and decision-making.

Tower of Hanoi is a simplified educational model. A real robotic stacking system would require additional considerations such as weight limits, stability, container dimensions, collision avoidance, and robot movement restrictions.

## Limitations and Future Enhancements

- Verify and expand tests for A* optimality and edge cases.
- Validate every container constraint against the CSP implementation.
- Improve performance comparisons across larger problem instances.
- Add a visual representation of states and movement sequences.
- Extend the model with physical constraints and realistic robot movement.
- Improve input validation and error handling.

## Conclusion

This project demonstrates how search algorithms and AI planning techniques can solve simplified movement and object-arrangement problems. It combines Tower of Hanoi search with planning examples involving STRIPS-style actions, Blocks World, CSP-based container stacking, and robotic action sequences.

The project provides an educational foundation for understanding how AI search and planning can be applied to more complex robotic and warehouse organization problems.

## Repository

GitHub: https://github.com/jananie052008-source/Tower-of-Hanoi-AI

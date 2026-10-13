
import heapq
from collections import deque


class HanoiStateSpace:
    """State-space model for the Tower of Hanoi problem."""

    def __init__(self, num_disks=3):
        if num_disks < 1:
            raise ValueError("Number of disks must be at least 1")

        self.num_disks = num_disks
        # Each peg is a tuple ordered from bottom to top.
        self.start_state = (
            tuple(range(num_disks, 0, -1)),
            (),
            (),
        )
        self.goal_state = (
            (),
            (),
            tuple(range(num_disks, 0, -1)),
        )

    def get_valid_moves(self, state):
        """Return all legal moves as (successor_state, description)."""
        moves = []

        for source in range(3):
            if not state[source]:
                continue

            disk = state[source][-1]

            for destination in range(3):
                if source == destination:
                    continue

                # A larger disk cannot be placed on a smaller disk.
                if state[destination] and state[destination][-1] < disk:
                    continue

                new_state = [list(peg) for peg in state]
                new_state[source].pop()
                new_state[destination].append(disk)
                successor = tuple(tuple(peg) for peg in new_state)

                description = (
                    f"move disk {disk} from "
                    f"{'ABC'[source]} to {'ABC'[destination]}"
                )
                moves.append((successor, description))

        return moves

    def heuristic(self, state):
        """Admissible heuristic: disks not yet on the goal peg."""
        return self.num_disks - len(state[2])

    def render_pegs(self, state):
        """Return a readable representation of the three pegs."""
        return {
            "A": list(state[0]),
            "B": list(state[1]),
            "C": list(state[2]),
        }


def breadth_first_search(problem):
    """Find an optimal solution using BFS."""
    start = problem.start_state
    goal = problem.goal_state

    queue = deque([(start, [])])
    visited = {start}
    nodes_expanded = 0

    while queue:
        current, path = queue.popleft()
        nodes_expanded += 1

        if current == goal:
            return path, nodes_expanded

        for successor, move in problem.get_valid_moves(current):
            if successor not in visited:
                visited.add(successor)
                queue.append(
                    (successor, path + [(move, successor)])
                )

    return None, nodes_expanded


def a_star_search(problem):
    """Find a solution using A* search."""
    start = problem.start_state
    goal = problem.goal_state

    counter = 0
    queue = [(problem.heuristic(start), counter, start, [], 0)]
    best_cost = {start: 0}
    nodes_expanded = 0

    while queue:
        _, _, current, path, cost = heapq.heappop(queue)

        if cost != best_cost.get(current):
            continue

        nodes_expanded += 1

        if current == goal:
            return path, nodes_expanded

        for successor, move in problem.get_valid_moves(current):
            new_cost = cost + 1

            if new_cost < best_cost.get(successor, float("inf")):
                best_cost[successor] = new_cost
                counter += 1
                priority = new_cost + problem.heuristic(successor)

                heapq.heappush(
                    queue,
                    (
                        priority,
                        counter,
                        successor,
                        path + [(move, successor)],
                        new_cost,
                    ),
                )

    return None, nodes_expanded


def depth_first_search(problem):
    """Find a solution using DFS; it is not guaranteed to be optimal."""
    start = problem.start_state
    goal = problem.goal_state

    stack = [(start, [])]
    visited = {start}
    nodes_expanded = 0

    while stack:
        current, path = stack.pop()
        nodes_expanded += 1

        if current == goal:
            return path, nodes_expanded

        for successor, move in problem.get_valid_moves(current):
            if successor not in visited:
                visited.add(successor)
                stack.append(
                    (successor, path + [(move, successor)])
                )

    return None, nodes_expanded

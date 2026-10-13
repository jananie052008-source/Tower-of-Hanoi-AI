
from collections import deque
import heapq


class HanoiStateSpace:
    """Tower of Hanoi represented as a state-space search problem."""

    def __init__(self, num_disks=3):
        if not isinstance(num_disks, int) or isinstance(num_disks, bool) or num_disks < 1:
            raise ValueError("Number of disks must be a positive integer")

        self.num_disks = num_disks
        self.initial_state = (
            tuple(range(num_disks, 0, -1)),
            (),
            (),
        )
        self.goal_state = (
            (),
            (),
            tuple(range(num_disks, 0, -1)),
        )

    def get_successors(self, state):
        """Return every legal next state and the move that creates it."""
        successors = []

        for source in range(3):
            if not state[source]:
                continue

            disk = state[source][-1]

            for destination in range(3):
                if source == destination:
                    continue

                if state[destination] and state[destination][-1] < disk:
                    continue

                new_pegs = [list(peg) for peg in state]
                new_pegs[source].pop()
                new_pegs[destination].append(disk)

                next_state = tuple(tuple(peg) for peg in new_pegs)
                move = (disk, source, destination)
                successors.append((next_state, move))

        return successors


def bfs_search(problem):
    """Find an optimal solution using breadth-first search."""
    start = problem.initial_state
    goal = problem.goal_state

    queue = deque([start])
    parent = {start: (None, None)}

    while queue:
        state = queue.popleft()

        if state == goal:
            break

        for next_state, move in problem.get_successors(state):
            if next_state not in parent:
                parent[next_state] = (state, move)
                queue.append(next_state)

    if goal not in parent:
        return [], 0

    path = []
    state = goal

    while parent[state][0] is not None:
        previous_state, move = parent[state]
        path.append((move, state))
        state = previous_state

    path.reverse()
    return path, len(path)


def a_star_search(problem):
    """Find an optimal solution using A* search."""
    start = problem.initial_state
    goal = problem.goal_state

    def heuristic(state):
        # Each disk outside the goal peg must be moved at least once.
        return problem.num_disks - len(state[2])

    frontier = [(heuristic(start), 0, start)]
    parent = {start: (None, None)}
    cost_so_far = {start: 0}
    counter = 1

    while frontier:
        _, cost, state = heapq.heappop(frontier)

        if cost != cost_so_far[state]:
            continue

        if state == goal:
            break

        for next_state, move in problem.get_successors(state):
            new_cost = cost + 1

            if new_cost < cost_so_far.get(next_state, float("inf")):
                cost_so_far[next_state] = new_cost
                parent[next_state] = (state, move)

                priority = new_cost + heuristic(next_state)
                heapq.heappush(
                    frontier,
                    (priority, counter, next_state),
                )
                counter += 1

    if goal not in parent:
        return [], 0

    path = []
    state = goal

    while parent[state][0] is not None:
        previous_state, move = parent[state]
        path.append((move, state))
        state = previous_state

    path.reverse()
    return path, len(path)


if __name__ == "__main__":
    problem = HanoiStateSpace(num_disks=3)

    for name, search in (
        ("BFS", bfs_search),
        ("A*", a_star_search),
    ):
        path, cost = search(problem)
        print(f"\n{name} Search")
        print(f"Number of moves: {cost}")

        for step, (move, state) in enumerate(path, start=1):
            disk, source, destination = move
            print(
                f"{step}. Move disk {disk} "
                f"from peg {chr(65 + source)} "
                f"to peg {chr(65 + destination)}"
            )

        print(f"Goal reached: {bool(path) and path[-1][1] == problem.goal_state}")

        

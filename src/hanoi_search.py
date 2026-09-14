import heapq
from collections import deque

class HanoiStateSpace:
    """
    Models Tower of Hanoi as an explicit state-space search problem.
    Demonstrates state mapping: Peg configurations mapped directly to search graph nodes.
    """
    def __init__(self, num_disks=3):
        self.num_disks = num_disks
        self.start_state = (tuple(range(num_disks, 0, -1)), (), ())
        self.goal_state = ((), (), tuple(range(num_disks, 0, -1)))

    def get_valid_moves(self, state):
        """Generates legal state transitions from current state configuration."""
        successors = []
        for src in range(3):
            if not state[src]:
                continue
            disk_to_move = state[src][-1]
            for dst in range(3):
                if src == dst:
                    continue
                if not state[dst] or state[dst][-1] > disk_to_move:
                    new_state = list(list(p) for p in state)
                    new_state[src].pop()
                    new_state[dst].append(disk_to_move)
                    hashable_state = tuple(tuple(p) for p in new_state)
                    move_desc = f"Move disk {disk_to_move} from Peg {chr(65+src)} to Peg {chr(65+dst)}"
                    successors.append((hashable_state, move_desc))
        return successors

    def heuristic(self, state):
        """Admissible heuristic: count of disks not yet on target Peg C."""
        return self.num_disks - len(state[2])


def breadth_first_search(problem):
    start = problem.start_state
    goal = problem.goal_state
    queue = deque([(start, [])])
    visited = {start}
    nodes_expanded = 0

    while queue:
        current_state, path = queue.popleft()
        nodes_expanded += 1
        if current_state == goal:
            return path, nodes_expanded

        for successor, move in problem.get_valid_moves(current_state):
            if successor not in visited:
                visited.add(successor)
                queue.append((successor, path + [(move, successor)]))

    return None, nodes_expanded


def a_star_search(problem):
    start = problem.start_state
    goal = problem.goal_state
    counter = 0
    pq = [(problem.heuristic(start), counter, start, [], 0)]
    visited = {}
    nodes_expanded = 0

    while pq:
        f, _, current_state, path, g = heapq.heappop(pq)
        nodes_expanded += 1

        if current_state == goal:
            return path, nodes_expanded

        if current_state in visited and visited[current_state] <= g:
            continue
        visited[current_state] = g

        for successor, move in problem.get_valid_moves(current_state):
            new_g = g + 1
            if successor not in visited or new_g < visited[successor]:
                counter += 1
                f_score = new_g + problem.heuristic(successor)
                heapq.heappush(pq, (f_score, counter, successor, path + [(move, successor)], new_g))

    return None, nodes_expanded


if __name__ == "__main__":
    disks = 3
    hanoi = HanoiStateSpace(num_disks=disks)
    
    print(f"=== Tower of Hanoi State Space Search (Disks: {disks}) ===")
    
    bfs_path, bfs_nodes = breadth_first_search(hanoi)
    astar_path, astar_nodes = a_star_search(hanoi)
    
    print(f"\n[Search Comparison]")
    print(f"BFS  -> Solution Moves: {len(bfs_path)} | States Expanded: {bfs_nodes}")
    print(f"A*   -> Solution Moves: {len(astar_path)} | States Expanded: {astar_nodes}\n")

    print("State Transition Graph Traversal Path:")
    print(f"Start State: {hanoi.start_state}")
    for step, (move, state) in enumerate(astar_path, 1):
        print(f"Step {step}: {move} ==> New State: {state}")
        # Final state space search execution
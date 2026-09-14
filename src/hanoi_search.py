import heapq
import time
from collections import deque

class HanoiStateSpace:
    """
    Models Tower of Hanoi as an explicit state-space search problem.
    State representation: Tuple of 3 tuples (Peg A, Peg B, Peg C).
    """
    def __init__(self, num_disks=3):
        self.num_disks = num_disks
        self.start_state = (tuple(range(num_disks, 0, -1)), (), ())
        self.goal_state = ((), (), tuple(range(num_disks, 0, -1)))

    def get_valid_moves(self, state):
        """Generates legal state transitions from current configuration."""
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

    def render_pegs(self, state):
        """Formats the tuple state into a clean string representation."""
        return f"A: {list(state[0])} | B: {list(state[1])} | C: {list(state[2])}"


def breadth_first_search(problem):
    """Uninformed Breadth-First Search."""
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
    """Informed A* Search using displacement heuristic."""
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
    
    print("=================================================================")
    print(f"   TOWER OF HANOI STATE-SPACE SEARCH (Disks: {disks})")
    print("=================================================================\n")
    
    # Run Algorithms
    bfs_path, bfs_nodes = breadth_first_search(hanoi)
    astar_path, astar_nodes = a_star_search(hanoi)
    
    # 1. Search Metric Comparison
    print("[1. Algorithm Comparison]")
    print(f"BFS (Uninformed) -> Moves: {len(bfs_path)} | Expanded States: {bfs_nodes}")
    print(f"A*  (Informed)   -> Moves: {len(astar_path)} | Expanded States: {astar_nodes}\n")

    # 2. Step-by-step Solution Traversal
    print("[2. Step-by-Step State Transition Path]")
    print(f"Start State : {hanoi.render_pegs(hanoi.start_state)}")
    for step, (move, state) in enumerate(astar_path, 1):
        print(f"Step {step:<2}: {move:<32} ==> State: {hanoi.render_pegs(state)}")
    
    # 3. Scaling Performance Benchmark
    print("\n[3. Performance Benchmark Across Disk Configurations]")
    print(f"{'Disks':<6}| {'Algorithm':<10}| {'Moves':<8}| {'Nodes Expanded':<16}| {'Time (s)':<10}")
    print("-" * 55)

    for d in [3, 4, 5]:
        p = HanoiStateSpace(num_disks=d)
        
        t0 = time.perf_counter()
        b_path, b_nodes = breadth_first_search(p)
        t_bfs = time.perf_counter() - t0
        print(f"{d:<6}| {'BFS':<10}| {len(b_path):<8}| {b_nodes:<16}| {t_bfs:.5f}")

        t0 = time.perf_counter()
        a_path, a_nodes = a_star_search(p)
        t_astar = time.perf_counter() - t0
        print(f"{d:<6}| {'A*':<10}| {len(a_path):<8}| {a_nodes:<16}| {t_astar:.5f}")
        print("-" * 55)
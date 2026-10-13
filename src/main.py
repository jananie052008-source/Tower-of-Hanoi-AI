"""Tower of Hanoi -> Robotic Container-Stacking Planner.
Includes BFS/DFS, STRIPS planning, CSP backtracking + forward checking/MRV,
robot-arm safety checks, explanations, and small comparison experiments.
"""
import random
import time
from collections import deque
from dataclasses import dataclass

# Keep this as your own assigned register number.
REG_NO = 15201
N_DISKS = 3 + sum(map(int, str(REG_NO))) % 3
PEGS = ("A", "B", "C")
ARM_LIMIT_DEFAULT = 10

# -------------------- 1. Tower of Hanoi: BFS and DFS --------------------
class TowerOfHanoi:
    def __init__(self):
        rng = random.Random(REG_NO)
        self.moves = [(i, j) for i in range(3) for j in range(3) if i != j]
        rng.shuffle(self.moves)
        self.start = (tuple(range(N_DISKS, 0, -1)), (), ())
        self.goal = ((), (), tuple(range(N_DISKS, 0, -1)))

    def neighbors(self, state):
        for src, dst in self.moves:
            if state[src] and (not state[dst] or state[dst][-1] > state[src][-1]):
                new = [list(p) for p in state]
                disk = new[src].pop()
                new[dst].append(disk)
                yield tuple(tuple(p) for p in new), (src, dst, disk)

    def search(self, mode="bfs"):
        if mode not in ("bfs", "dfs"):
            raise ValueError("mode must be 'bfs' or 'dfs'")
        frontier = deque([self.start])
        parent = {self.start: (None, None)}
        expanded = 0
        while frontier:
            state = frontier.popleft() if mode == "bfs" else frontier.pop()
            expanded += 1
            if state == self.goal:
                path = []
                while parent[state][0] is not None:
                    state, move = parent[state]
                    path.append(move)
                return path[::-1], expanded
            for nxt, move in self.neighbors(state):
                if nxt not in parent:
                    parent[nxt] = (state, move)
                    frontier.append(nxt)
        return None, expanded

# -------------------- 2. STRIPS planner and explanations --------------------
@dataclass(frozen=True)
class Action:
    name: str
    args: tuple
    pre: frozenset
    add: frozenset
    delete: frozenset

    def __str__(self):
        return f"{self.name}({', '.join(self.args)})"


def applicable(state, action):
    return action.pre <= state


def apply_action(state, action):
    if not applicable(state, action):
        raise ValueError(f"Illegal action {action}: preconditions are not satisfied")
    return (state - action.delete) | action.add


def plan(initial, goal, actions):
    """BFS over logical fact sets. Returns (actions, expanded_nodes), or (None, nodes)."""
    start, goal = frozenset(initial), frozenset(goal)
    frontier, parent = deque([start]), {start: None}
    expanded = 0
    while frontier:
        state = frontier.popleft()
        expanded += 1
        if goal <= state:
            path = []
            while parent[state] is not None:
                state, action = parent[state]
                path.append(action)
            return path[::-1], expanded
        for action in actions:
            if applicable(state, action):
                nxt = apply_action(state, action)
                if nxt not in parent:
                    parent[nxt] = (state, action)
                    frontier.append(nxt)
    return None, expanded


def explain(i, action):
    fmt = lambda facts: ", ".join(f"{p[0]}({','.join(p[1:])})" for p in sorted(facts))
    return (f"Step {i}: {action}\n  legal because: {fmt(action.pre)}\n"
            f"  effects: +[{fmt(action.add)}] -[{fmt(action.delete)}]")

# -------------------- 3. Hanoi and Blocks World action domains --------------------
def hanoi_domain(n):
    if n < 1:
        raise ValueError("Hanoi requires at least one disk")
    disks = [f"d{i}" for i in range(1, n + 1)]  # d1 is smallest
    pegs = list(PEGS)
    size = {d: i for i, d in enumerate(disks, 1)}
    supports, actions = disks + pegs, []
    for d in disks:
        for src in supports:
            for dst in supports:
                if src == dst or d in (src, dst):
                    continue
                if dst in size and size[dst] <= size[d]:
                    continue
                if src in size and size[src] <= size[d]:
                    continue
                actions.append(Action("Move", (d, src, dst),
                    frozenset({("On", d, src), ("Clear", d), ("Clear", dst)}),
                    frozenset({("On", d, dst), ("Clear", src)}),
                    frozenset({("On", d, src), ("Clear", dst)})))
    init = {("On", disks[-1], "A"), ("Clear", "B"), ("Clear", "C"), ("Clear", disks[0])}
    goal = {("On", disks[-1], "C")}
    for big, small in zip(disks[::-1], disks[::-1][1:]):
        init.add(("On", small, big))
        goal.add(("On", small, big))
    return init, goal, actions


def blocks_domain(initial_stacks, goal_stacks, weights=None, arm_limit=ARM_LIMIT_DEFAULT):
    """Stacks are bottom-to-top lists. Any action that lifts x is blocked if x is too heavy."""
    if arm_limit < 0:
        raise ValueError("arm_limit cannot be negative")
    items = [x for stack in initial_stacks for x in stack]
    if len(items) != len(set(items)):
        raise ValueError("Each container must appear exactly once in initial_stacks")
    if {x for stack in goal_stacks for x in stack} != set(items):
        raise ValueError("Goal stacks must contain every initial container exactly once")
    weights = {x: 1 for x in items} if weights is None else dict(weights)
    if any(x not in weights or weights[x] < 0 for x in items):
        raise ValueError("Provide a non-negative weight for every container")

    actions = []
    for x in items:
        # SAFETY: Pickup and Unstack both lift x, so neither exists for an overweight x.
        if weights[x] > arm_limit:
            continue
        actions.append(Action("Pickup", (x,),
            frozenset({("OnTable", x), ("Clear", x), ("ArmEmpty",)}),
            frozenset({("Holding", x)}),
            frozenset({("OnTable", x), ("Clear", x), ("ArmEmpty",)})))
        actions.append(Action("Putdown", (x,), frozenset({("Holding", x)}),
            frozenset({("OnTable", x), ("Clear", x), ("ArmEmpty",)}),
            frozenset({("Holding", x)})))
        for y in items:
            if x == y:
                continue
            actions.append(Action("Stack", (x, y),
                frozenset({("Holding", x), ("Clear", y)}),
                frozenset({("On", x, y), ("Clear", x), ("ArmEmpty",)}),
                frozenset({("Holding", x), ("Clear", y)})))
            # Unstack lifts x; generated only when x is within the arm limit.
            actions.append(Action("Unstack", (x, y),
                frozenset({("On", x, y), ("Clear", x), ("ArmEmpty",)}),
                frozenset({("Holding", x), ("Clear", y)}),
                frozenset({("On", x, y), ("Clear", x), ("ArmEmpty",)})))

    def facts(stacks, include_arm=True):
        result = set()
        for stack in stacks:
            if stack:
                result.add(("OnTable", stack[0]))
                result.add(("Clear", stack[-1]))
                result.update(("On", hi, lo) for lo, hi in zip(stack, stack[1:]))
        if include_arm:
            result.add(("ArmEmpty",))
        return result

    def goal_facts(stacks):
        result = set()
        for stack in stacks:
            if stack:
                result.add(("OnTable", stack[0]))
                result.update(("On", hi, lo) for lo, hi in zip(stack, stack[1:]))
        return result

    return facts(initial_stacks), goal_facts(goal_stacks), actions

# -------------------- 4. CSP: assign containers to stacks --------------------
@dataclass(frozen=True)
class Container:
    name: str
    weight: int
    departure: int  # smaller value = leaves earlier
    hazardous: bool = False


def compatible(a, b):
    # No two hazardous containers share a stack; heavier must be below later-departing.
    if a.hazardous and b.hazardous:
        return False
    return (a.weight - b.weight) * (a.departure - b.departure) >= 0


def solve(containers, n_stacks, max_height, forward_check=True, mrv=False):
    if n_stacks < 1 or max_height < 1:
        raise ValueError("n_stacks and max_height must be positive")
    names = [c.name for c in containers]
    if len(names) != len(set(names)):
        raise ValueError("Container names must be unique")
    stats = {"assignments_tried": 0, "backtracks": 0}
    if len(containers) > n_stacks * max_height or sum(c.hazardous for c in containers) > n_stacks:
        return None, stats
    by_name = {c.name: c for c in containers}
    domains = {name: set(range(n_stacks)) for name in names}
    stacks, assignment = {s: [] for s in range(n_stacks)}, {}

    def consistent(c, s):
        return len(stacks[s]) < max_height and all(compatible(c, other) for other in stacks[s])

    def choose():
        left = [n for n in names if n not in assignment]
        return min(left, key=lambda n: (len(domains[n]), n)) if mrv and left else (left[0] if left else None)

    def backtrack():
        name = choose()
        if name is None:
            return True
        c = by_name[name]
        for s in sorted(domains[name]):
            if not consistent(c, s):
                continue
            stats["assignments_tried"] += 1
            assignment[name] = s
            stacks[s].append(c)
            removed, ok = [], True
            if forward_check:
                for other in names:
                    if other in assignment or s not in domains[other]:
                        continue
                    if not consistent(by_name[other], s):
                        domains[other].remove(s)
                        removed.append(other)
                        if not domains[other]:
                            ok = False
                            break
            if ok and backtrack():
                return True
            for other in removed:
                domains[other].add(s)
            stacks[s].pop()
            del assignment[name]
            stats["backtracks"] += 1
        return False

    return (dict(assignment) if backtrack() else None), stats


def to_stacks(containers, assignment, n_stacks):
    by_name = {c.name: c for c in containers}
    result = [[] for _ in range(n_stacks)]
    for name, stack_id in assignment.items():
        result[stack_id].append(by_name[name])
    return [[c.name for c in sorted(s, key=lambda c: (-c.weight, -c.departure))] for s in result]


def generate_containers(n, seed=REG_NO):
    rng = random.Random(seed)
    return [Container(f"C{i+1}", rng.randint(1, 9), rng.randint(1, 6), rng.random() < 0.2)
            for i in range(n)]

# -------------------- 5. Demonstrations and checks --------------------
def show_robot_plan(init, goal, actions, weights, arm_limit):
    steps, nodes = plan(init, goal, actions)
    if steps is None:
        print("No safe plan exists under the current arm limit.")
        return
    print(f"\nRobot plan: {len(steps)} actions; {nodes} states expanded")
    state = frozenset(init)
    for i, action in enumerate(steps, 1):
        # Second safety gate: reject any plan step that attempts to lift an overweight item.
        if action.name in ("Pickup", "Unstack") and weights[action.args[0]] > arm_limit:
            raise RuntimeError(f"Safety stop: arm cannot lift {action.args[0]}")
        print(explain(i, action))
        state = apply_action(state, action)
    if not frozenset(goal) <= state:
        raise RuntimeError("Planner ended without satisfying the goal")
    print("Goal verified: all required stack relations are satisfied.")


def demo_hanoi_same_planner():
    init, goal, actions = hanoi_domain(N_DISKS)
    steps, nodes = plan(init, goal, actions)
    base, base_nodes = TowerOfHanoi().search("bfs")
    if steps is None or base is None:
        print("Hanoi planner failed to find a solution.")
        return
    print(f"Hanoi via STRIPS: {len(steps)} moves, {nodes} nodes")
    print(f"Hanoi via BFS:    {len(base)} moves, {base_nodes} nodes (lengths should match)\n")


def demo_blocks_sizes():
    print("Blocks World scaling: all blocks on table -> one tower")
    print(f"{'blocks':>6} {'plan':>6} {'nodes':>8}")
    for n in (3, 4, 5):
        names = [f"B{i}" for i in range(1, n + 1)]
        init, goal, actions = blocks_domain([[x] for x in names], [names])
        steps, nodes = plan(init, goal, actions)
        print(f"{n:>6} {len(steps) if steps is not None else 'N/A':>6} {nodes:>8}")
    print()


def demo_csp_compare():
    print("CSP comparison: plain backtracking vs forward checking/MRV")
    print(f"{'n':>3} {'stacks':>6} {'plain tried':>11} {'FC tried':>9} {'FC+MRV':>8} | {'plain s':>8} {'FC s':>8} {'MRV s':>8}")
    for n, ns, height in ((12, 4, 3), (18, 5, 4), (18, 6, 3)):
        cs, results = generate_containers(n), []
        for fc, mrv in ((False, False), (True, False), (True, True)):
            start = time.perf_counter()
            sol, stats = solve(cs, ns, height, fc, mrv)
            results.append((stats["assignments_tried"], time.perf_counter() - start, sol is not None))
        print(f"{n:>3} {ns:>6} {results[0][0]:>11} {results[1][0]:>9} {results[2][0]:>8} | "
              f"{results[0][1]:>8.4f} {results[1][1]:>8.4f} {results[2][1]:>8.4f} | solved={all(r[2] for r in results)}")
    print()


def demo_pipeline():
    cs = generate_containers(6)
    print("Container manifest (weight, departure, hazardous):")
    for c in cs:
        print(f"  {c.name}: weight={c.weight}, departure={c.departure}, hazardous={c.hazardous}")
    assignment, stats = solve(cs, 3, 3, forward_check=True, mrv=True)
    if assignment is None:
        print("No valid CSP assignment found; increase stack capacity or change constraints.")
        return
    goal_stacks = to_stacks(cs, assignment, 3)
    print("\nCSP stack assignment (bottom -> top):", goal_stacks)
    print("CSP stats:", stats)
    weights = {c.name: c.weight for c in cs}
    arm_limit = 9
    init, goal, actions = blocks_domain([[c.name] for c in cs], goal_stacks, weights, arm_limit)
    show_robot_plan(init, goal, actions, weights, arm_limit)


if __name__ == "__main__":
    game = TowerOfHanoi()
    bfs, bfs_nodes = game.search("bfs")
    dfs, dfs_nodes = game.search("dfs")
    print(f"Register: {REG_NO} | Disks: {N_DISKS} | States: {3 ** N_DISKS}")
    print(f"BFS: {len(bfs)} moves (optimal), nodes={bfs_nodes}")
    print(f"DFS: {len(dfs)} moves, nodes={dfs_nodes}\n")
    print("Shortest solution (BFS):")
    for i, (src, dst, disk) in enumerate(bfs, 1):
        print(f"Step {i}: move disk {disk} from {PEGS[src]} to {PEGS[dst]}")
    print("\n" + "=" * 58 + "\nROBOTIC CONTAINER-STACKING PLANNER\n" + "=" * 58)
    demo_hanoi_same_planner()
    demo_blocks_sizes()
    demo_csp_compare()
    demo_pipeline()
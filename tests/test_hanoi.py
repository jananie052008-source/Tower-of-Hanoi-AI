import sys
import os
import unittest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from hanoi_search import HanoiStateSpace, breadth_first_search, a_star_search

class TestHanoiSearch(unittest.TestCase):

    def test_bfs_optimal_moves(self):
        """Verify BFS returns optimal 7 moves for N=3."""
        problem = HanoiStateSpace(num_disks=3)
        path, _ = breadth_first_search(problem)
        self.assertEqual(len(path), 7)

    def test_astar_optimal_moves(self):
        """Verify A* returns optimal 7 moves for N=3."""
        problem = HanoiStateSpace(num_disks=3)
        path, _ = a_star_search(problem)
        self.assertEqual(len(path), 7)

    def test_goal_state_reached(self):
        """Verify solution path arrives at the exact goal state."""
        problem = HanoiStateSpace(num_disks=3)
        path, _ = a_star_search(problem)
        final_state = path[-1][1]
        self.assertEqual(final_state, problem.goal_state)

if __name__ == '__main__':
    unittest.main()
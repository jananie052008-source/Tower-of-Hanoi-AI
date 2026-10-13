
import sys
import unittest
from pathlib import Path

# Make the src folder available to Python.
PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_FOLDER = PROJECT_ROOT / "src"
sys.path.insert(0, str(SRC_FOLDER))

from hanoi_search import HanoiStateSpace, a_star_search


class TestHanoiSearch(unittest.TestCase):

    def test_astar_finds_optimal_solution(self):
        problem = HanoiStateSpace(num_disks=3)
        path, expanded = a_star_search(problem)

        self.assertIsNotNone(path)
        self.assertEqual(len(path), 7)
        self.assertGreater(expanded, 0)

    def test_goal_state_is_reached(self):
        problem = HanoiStateSpace(num_disks=3)
        path, _ = a_star_search(problem)

        final_state = path[-1][1]
        self.assertEqual(final_state, problem.goal_state)

    def test_invalid_disk_count(self):
        with self.assertRaises(ValueError):
            HanoiStateSpace(num_disks=0)


if __name__ == "__main__":
    unittest.main()

import unittest

from arc_utilities.algorithms import is_list_unique


class TestAlgorithms(unittest.TestCase):

    def test_is_lis_unique(self):
        self.assertTrue(is_list_unique([1, 2, 3]))
        self.assertTrue(is_list_unique(['a', 2, 3]))
        self.assertTrue(is_list_unique(['a', 2, None]))

        self.assertFalse(is_list_unique(['a', 2, None, 2]))
        self.assertFalse(is_list_unique(['a', 'a', None, 2]))
        self.assertFalse(is_list_unique([8, 'a', None, None]))


if __name__ == '__main__':
    unittest.main()

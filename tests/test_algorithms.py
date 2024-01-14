import unittest

from arm_utilities.algorithms import is_list_unique, nested_dict_update, repeat_last, zip_repeat_shorter, \
    chunked_drop_last, reversed_chunked


class TestAlgorithms(unittest.TestCase):

    def test_is_list_unique(self):
        self.assertTrue(is_list_unique([1, 2, 3]))
        self.assertTrue(is_list_unique(['a', 2, 3]))
        self.assertTrue(is_list_unique(['a', 2, None]))

        self.assertFalse(is_list_unique(['a', 2, None, 2]))
        self.assertFalse(is_list_unique(['a', 'a', None, 2]))
        self.assertFalse(is_list_unique([8, 'a', None, None]))

    def test_nested_dict_update(self):
        base_dict = {
            'a': 1,
            'b': [2, 3],
            'd': [2, 3],
            'c': {
                'x': "x",
                'y': "y",
            },
            'e': None,
        }

        update_dict = {
            'a': 100,
            'd': [4, 5, 6],
            'c': {
                'x': "not_x",
            },
            'f': 1,
        }
        updated_dict = nested_dict_update(base_dict, update_dict)

        self.assertEqual(updated_dict['a'], 100)
        self.assertEqual(updated_dict['b'], [2, 3])
        self.assertEqual(updated_dict['c']['x'], "not_x")
        self.assertEqual(updated_dict['c']['y'], "y")
        self.assertEqual(updated_dict['d'], [4, 5, 6])
        self.assertEqual(updated_dict['e'], None)
        self.assertEqual(updated_dict['f'], 1)

        updated_dict = nested_dict_update(base_dict, None)

        self.assertEqual(updated_dict, base_dict)

    def test_repeat_last(self):
        i = repeat_last([1, 2])
        self.assertEqual(next(i), 1)
        self.assertEqual(next(i), 2)
        self.assertEqual(next(i), 2)

    def test_zip_repeat_shorter(self):
        i = zip_repeat_shorter([1, 2], ['a', 'b', 'c'])
        self.assertEqual(next(i), (1, 'a'))
        self.assertEqual(next(i), (2, 'b'))
        self.assertEqual(next(i), (2, 'c'))
        with self.assertRaises(StopIteration):
            next(i)

        i = zip_repeat_shorter([1, 2, 3], ['a', 'b'])
        self.assertEqual(next(i), (1, 'a'))
        self.assertEqual(next(i), (2, 'b'))
        self.assertEqual(next(i), (3, 'b'))
        with self.assertRaises(StopIteration):
            next(i)

    def test_chunked_drop_last(self):
        self.assertEqual(list(chunked_drop_last(range(10), 5)), [[0, 1, 2, 3, 4], [5, 6, 7, 8, 9]])
        self.assertEqual(list(chunked_drop_last(range(5), 5)), [[0, 1, 2, 3, 4]])
        self.assertEqual(list(chunked_drop_last(range(6), 5)), [[0, 1, 2, 3, 4]])

    def test_reversed_chunked(self):
        self.assertEqual(reversed_chunked(list(range(10)), 5), [[0, 1, 2, 3, 4], [5, 6, 7, 8, 9]])
        self.assertEqual(reversed_chunked(list(range(5)), 5), [[0, 1, 2, 3, 4]])
        self.assertEqual(reversed_chunked(list(range(6)), 5), [[1, 2, 3, 4, 5]])


if __name__ == '__main__':
    unittest.main()

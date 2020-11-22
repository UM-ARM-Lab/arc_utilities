import unittest

from arc_utilities.conditional_try import conditional_try


def _raises(s):
    raise ValueError("error: " + s)


def _not_raises(s):
    return "no error: " + s


class TestConditionalTry(unittest.TestCase):

    def test_raises(self):
        with self.assertRaises(ValueError):
            conditional_try(False, _raises, s="test")

    def test_no_raise(self):
        conditional_try(False, _not_raises, s="test")

        self.assertTrue(True)

    def test_no_raises_catch(self):
        conditional_try(True, _raises, s="test")
        conditional_try(True, _not_raises, s="test")

        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()

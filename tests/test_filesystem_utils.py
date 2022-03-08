#! /usr/bin/env python
import os
import pathlib
import unittest

from arc_utilities.filesystem_utils import no_overwrite_path


def _try_unlink(path_str):
    if os.path.exists(path_str):
        os.unlink(path_str)


class TestFilesystemUtils(unittest.TestCase):

    def setUp(self) -> None:
        self.dir = pathlib.Path('.test')
        self.dir.mkdir(exist_ok=True)
        _try_unlink(".test/no_overwrite")
        _try_unlink(".test/no_overwrite_1")
        _try_unlink(".test/no_overwrite.txt")
        _try_unlink(".test/no_overwrite_1.txt")
        _try_unlink(".test/no_overwrite.txt.gz")
        _try_unlink(".test/no_overwrite_1.txt.gz")

    def test_no_overwrite_path(self):
        path = no_overwrite_path(self.dir / 'no_overwrite')
        self.assertEqual(path.as_posix(), '.test/no_overwrite')
        path.touch()

        path = no_overwrite_path(self.dir / 'no_overwrite')
        self.assertEqual(path.as_posix(), '.test/no_overwrite_v1')

        path = no_overwrite_path(self.dir / 'no_overwrite.txt')
        self.assertEqual(path.as_posix(), '.test/no_overwrite.txt')
        path.touch()

        path = no_overwrite_path(self.dir / 'no_overwrite.txt')
        self.assertEqual(path.as_posix(), '.test/no_overwrite_v1.txt')

        path = no_overwrite_path(self.dir / 'no_overwrite.txt.gz')
        self.assertEqual(path.as_posix(), '.test/no_overwrite.txt.gz')
        path.touch()

        path = no_overwrite_path(self.dir / 'no_overwrite.txt.gz')
        self.assertEqual(path.as_posix(), '.test/no_overwrite_v1.txt.gz')


if __name__ == '__main__':
    unittest.main()

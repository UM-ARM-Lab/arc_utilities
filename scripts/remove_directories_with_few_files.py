#!/usr/bin/env python
import argparse
import pathlib

import colorama

from arc_utilities.filesystem_utils import ask_to_remove_directories, count_files_recursive


def main():
    colorama.init(autoreset=True)
    parser = argparse.ArgumentParser("finds directories smaller than a given size, asks for confirmation, then deletes")
    parser.add_argument('root', type=pathlib.Path, help="root directory", nargs='+')
    parser.add_argument('count', type=int, help="remove the dir if there we fewer than this many files/subfilders")

    args = parser.parse_args()

    directories_to_remove = []
    for root in args.root:
        for d in root.iterdir():
            if d.is_dir():
                n = count_files_recursive(d)
                if n < args.count:
                    directories_to_remove.append(d)

    ask_to_remove_directories(directories_to_remove)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import argparse
import logging
import os
import sys
from pathlib import Path

import pymavlink

from muas_sid import __version__

module = sys.modules["__main__"].__file__
logger = logging.getLogger(module)


def existing_file(value: str, extensions: list = None) -> Path:
    """Check object is an existing file

    Arguments:
        value {str} -- File path
    
    Keyword Arguments:
        extensions {list} -- Possible file extensions (default: {None})

    Raises:
        IOError: Not an existing file
        argparse.ArgumentTypeError: Exists, but is not the correct file type

    Returns:
        Path -- Path object to file
    """

    file_path = Path(value).expanduser()
    if not os.path.isfile(file_path):
        raise IOError("{} does not exist".format(file_path))
    if extensions is not None:
        suffixes = [ext.lower() for ext in extensions]
        if file_path.suffix.lower() not in suffixes:
            raise argparse.ArgumentTypeError(
                "{} is not a file of type {}".format(value, extensions)
            )

    return file_path


def existing_directory(value: str) -> Path:
    """Check object is an existing directory

    Arguments:
        value {str} -- Directory path

    Raises:
        IOError: Not an existing directory 
        argparse.ArgumentTypeError: Exists, but is not a directory

    Returns:
        Path -- Path object to directory
    """
    directory_path = Path(value).expanduser()
    if not directory_path.exists():
        raise IOError("{} does not exists".format(value))

    if not directory_path.is_dir():
        raise argparse.ArgumentTypeError("{} is not a directory".format(value))

    return directory_path


def parse_command_line(argv: list = None) -> argparse.Namespace:
    """Handle command line arguments

    Keyword Arguments:
        argv {list} -- Vector of arguments (default: {None})

    Returns:
        argparse.Namespace -- Parsed arguments
    """

    if argv is None:
        argv = sys.argv

    formatter_class = argparse.RawTextHelpFormatter
    parser = argparse.ArgumentParser(
        description=__package__, formatter_class=formatter_class
    )

    parser.add_argument(
        "log_path",
        action="store",
        type=lambda f: existing_file(f, [".bin"]),
        help="Path to .bin log file",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        required=False,
        help="Increase log verbosity (max -vvv)",
        dest="verbose_count",
    )
    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        required=False,
        help="Show debugging messages (eqv. to -vv, overrides verbosity flag)",
    )
    parser.add_argument(
        "-s",
        "--silent",
        action="store_true",
        required=False,
        help="Suppress log warning and lower messages (overrides other verbosity flags)",
    )

    parser.add_argument(
        "-V",
        "--version",
        action="version",
        version="%(prog)s {}".format(__version__),
        help="show the version and exit",
    )

    args = parser.parse_args()
    if args.silent:
        logger.setLevel(logging.ERROR)
    elif args.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(max(3 - args.verbose_count, 1) * 10)

    return args

#!/usr/bin/env python3

import logging
import sys
import typing

from muas_sid.cli import parse_command_line
from muas_sid.read_log import load_bin, load_prev_obj

module = sys.modules["__main__"].__file__
logger = logging.getLogger(module)

if __name__ == "__main__":
    logging.basicConfig(
        format="%(name)s [%(levelname)s] %(message)s",
        level=logging.INFO,
        stream=sys.stderr,
    )

    args = parse_command_line()

    if args.cache is not True:
        logger.debug("Reading log file from {}".format(args.input_path))
        log_data = load_bin(args.input_path, args.cache_path)
    else:
        logger.info("Cache option set. Reading log py obj from previous run.")
        log_data = load_prev_obj(args.cache_path)

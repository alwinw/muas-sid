import logging
import sys

from muas_sid.cli import parse_command_line
from muas_sid.read_log import load_bin

module = sys.modules["__main__"].__file__
logger = logging.getLogger(module)

if __name__ == "__main__":
    logging.basicConfig(
        format="%(name)s [%(levelname)s] %(message)s",
        level=logging.INFO,
        stream=sys.stderr,
    )

    args = parse_command_line()

    logger.debug("Reading log file from {}".format(args.input_path))

    load_bin(args.input_path)

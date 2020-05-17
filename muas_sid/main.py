from muas_sid.cli import parse_command_line
import sys
import logging

module = sys.modules["__main__"].__file__
logger = logging.getLogger(module)

if __name__ == "__main__":
    logging.basicConfig(
        format="%(name)s [%(levelname)s] %(message)s",
        level=logging.INFO,
        stream=sys.stderr,
    )

    args = parse_command_line()

    logger.debug("Reading log file from {}".format(args.log_path))

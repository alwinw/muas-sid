
import fnmatch
import logging
import struct
import sys
import typing
from pathlib import Path

from pymavlink.DFReader import DFMessage, DFReader_binary, DFReader_text
from pymavlink.mavutil import mavlink_connection

module = sys.modules["__main__"].__file__
logger = logging.getLogger(module)

def load_bin(input_path:Path, types:set = None):
    m_log = mavlink_connection(device=str(input_path))  # type:DFReader_binary
    logger.info("Opened mavlink file connection to {}".format(input_path))

    # TODO: Type hint will fail if .log file as it will be type:DFReader_text
    # is_bin = input_path.suffix.lower() in ('.bin')
    # in_log = input_path.suffix.lower() in ('.log', '.tlog')

    if types is None:
        types = {'IMU', 'NKF1', 'NKF2', 'AOA', 'ARSP', 'BAT', 'RCOU'}

    if hasattr(m_log, 'name_to_id'):
        avail_types = set(m_log.name_to_id.keys())
        if not types.issubset(avail_types):
            match_types = types.intersection(avail_types)
            logger.warn("Some requested types not found in the log. Removed {}".format(types - match_types))
        else:
            logger.debug("All types checked and found in {}".format(input_path))
            match_types = types.union({"FMT"})
    else:
        logger.debug("Could not get available types from {}".format(input_path))
        match_types = types.union({"FMT"})

    output = dict.fromkeys(match_types)

    while True:
        m = m_log.recv_match(blocking=False, type=match_types)  # type:DFMessage
        if m is None:
            break

        msg_type = m.get_type()

        if msg_type == "FMT":
            if m.Name in match_types:
                output[m.Name] = {k: [] for k in ["timestamp"] + m.Columns.split(',')}
                logger.debug("{}: Found `{}` with columns `{}`".format(m._timestamp, m.Name, m.Columns))
        elif m.get_type() == 'BAD_DATA' and (m.reason == "Bad prefix"):
            continue
        else:
            msg_data = m.to_dict()
            # if 'data' in msg_data and type(msg_data['data']) is not dict:
            #     msg_data['data'] = list(msg_data['data'])
            msg_data["timestamp"] = m._timestamp
            for k in output[msg_type].keys():
                output[msg_type][k].append(msg_data[k])

    logger.info("Loaded required types from log file")

    return output.pop("FMT")

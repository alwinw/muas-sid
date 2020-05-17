#!/usr/bin/env python3

import logging
import pickle
import sys
from pathlib import Path

import pandas as pd
from pymavlink.DFReader import DFMessage, DFReader_binary, DFReader_text
from pymavlink.mavutil import mavlink_connection

module = sys.modules["__main__"].__file__
logger = logging.getLogger(module)


def save_current_obj(obj: dict, cache_path: Path) -> bool:
    with open(Path(cache_path).with_suffix(".1.dict"), 'wb') as dump_bin_file:
        pickle.dump(obj, dump_bin_file)
    return True


def load_prev_obj(cache_path: Path) -> dict:
    with open(Path(cache_path).with_suffix(".1.dict"), 'rb') as load_bin_file:
        obj = pickle.load(load_bin_file)
    return obj


def load_bin(input_path: Path, cache_path: Path, types: set = None) -> dict:
    """Load log file

    Arguments:
        input_path {Path} -- Path to log file
        cache_path {Path} -- Path to cache file

    Keyword Arguments:
        types {set} -- Desired message types (default: {None})

    Returns:
        dict -- Dict with types as keys and dict of messages
    """
    # TODO: Type hint will fail if .log file as it will be type:DFReader_text
    m_log = mavlink_connection(device=str(input_path))  # type:DFReader_binary
    logger.info("Opened mavlink file connection to {}".format(input_path))

    # Message types to be found
    if types is None:
        types = {'IMU', 'NKF1', 'NKF2', 'AOA', 'ARSP', 'BAT', 'RCOU'}

    if hasattr(m_log, 'name_to_id'):
        avail_types = set(m_log.name_to_id.keys())
        if not types.issubset(avail_types):
            match_types = types.intersection(avail_types)
            logger.warn("Some requested types not found in the log. Removed {}".format(
                types - match_types))
        else:
            logger.debug("All types checked and found in {}".format(input_path))
            match_types = types.union({"FMT"})
    else:
        logger.debug("Could not get available types from {}".format(input_path))
        match_types = types.union({"FMT"})

    # Traverse each message of type=match_types
    output = dict.fromkeys(match_types)
    log_timestamp = 0
    while True:
        m = m_log.recv_match(blocking=False, type=match_types)  # type:DFMessage
        if m is None:
            break

        msg_type = m.get_type()

        if msg_type == "FMT":
            if m.Name in match_types:
                output[m.Name] = {k: [] for k in ["timestamp"] + m.Columns.split(',')}
                logger.debug("{}: Found `{}` with columns `{}`".format(
                    m._timestamp, m.Name, m.Columns))
        elif m.get_type() == 'BAD_DATA' and (m.reason == "Bad prefix"):
            continue
        else:
            msg_data = m.to_dict()
            msg_data["timestamp"] = m._timestamp
            for k in output[msg_type].keys():
                output[msg_type][k].append(msg_data[k])
            if msg_data["timestamp"] - log_timestamp > 60:
                log_timestamp = msg_data["timestamp"]
                logger.debug("Processing {} message at {}".format(
                    msg_type, log_timestamp))

    output.pop("FMT")
    logger.info("Loaded required types from log file")

    output_pd = {key: pd.DataFrame.from_dict(output[key]) for key in output.keys()}

    save_current_obj(output_pd, cache_path)
    logger.debug("Saved result to cache file")

    return output_pd

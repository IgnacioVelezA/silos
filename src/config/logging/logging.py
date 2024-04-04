"""
logging.py:
    Functions required to configure logger.
"""
import json
import logging
import logging.config
import os

def create_folder(folder_path):
    """
    Creates new folder w.r.t. the current path
    """
    current_path = os.path.abspath(os.getcwd())
    directory = f"{current_path}/{folder_path}"
    if not os.path.exists(directory):
        os.makedirs(directory, 0o775)
        print(f"Successfully created new directory {directory}")

LOGGING_CONFIG_PATH = os.path.join('src',
                                    'config',
                                    'logging',
                                    'config_logging.json')
MAIN_LOGGER = '__main__'
PATH_TO_MAIN_LOGGER = os.path.join('src', 'logs', 'main.log')


def read_config(config_file_path, path_to_main_logger=PATH_TO_MAIN_LOGGER):
    """
    Returns logging configuration values.

    Parameters
    ----------
    config_file_path : str
        Path to JSON-format configuration file for logging.
        Path relative to src/ directory.

    Returns
    -------
    logging_config : dict
        Dictionary with values read from configuration file.
    """
    # create folder in case is not there
    create_folder(os.path.split(path_to_main_logger)[0])
    with open(config_file_path, 'r') as file:
        logging_config = json.load(file)

    return logging_config


def get_logger(logger_name=MAIN_LOGGER, config_file_path=LOGGING_CONFIG_PATH):
    """
    Returns logger object from logging configuration file.

    Parameters
    ----------
    logger_name : str, optional
        Name of logger in JSON configuration file.
    config_file_path : str, optional
        Path to JSON-format configuration file for logging.
        Path relative to src/ directory.
    Returns
    -------
    logger : logger object
        Logger loaded from JSON configuration file.
    """

    logging.config.dictConfig(read_config(config_file_path))
    logger = logging.getLogger(logger_name)

    return logger


def display_logger(path_to_log_file=PATH_TO_MAIN_LOGGER):
    """
    Open a new terminal to show logger messages while running scripts.

    Displayed messages are those contained in specified log file.

    Parameters
    ----------
    path_to_log_file : str
        Path to log file where logger messages are saved.

    Returns
    -------
    None.
    """
    bash_command = "gnome-terminal -- /bin/sh -c 'tail -F -n 1 {}'".format(path_to_log_file)
    os.system(bash_command)

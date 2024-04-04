import os
import can
from src.config.logging import logging

logger = logging.get_logger(logging.MAIN_LOGGER)

logger.info('setting up can0')
os.system('sudo ip link set can0 type can bitrate 1000000')
os.system('sudo ifconfig can0 up')
logger.info('can0 successfully set up')

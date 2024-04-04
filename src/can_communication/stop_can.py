import os
import can
from src.config.logging import logging

logger = logging.get_logger(logging.MAIN_LOGGER)
os.system('sudo ifconfig can0 down')
logger.info('can0 shut down')
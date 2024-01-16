import logging

class ColoredFormatter(logging.Formatter):
    COLORS = {
        'INFO': '\033[32m',    # Green for INFO
        'ERROR': '\033[31m',   # Red for ERROR
        'CRITICAL': '\033[31m',# Red for CRITICAL
        'DEBUG': '\033[36m',   # Cyan for DEBUG (you can customize)
        'WARNING': '\033[33m'  # Yellow for WARNING (you can customize)
    }

    RESET_COLOR = '\033[0m'

    def format(self, record):
        log_message = super().format(record)
        log_level = record.levelname

        if log_level in self.COLORS:
            return f"{self.COLORS[log_level]}{log_message}{self.RESET_COLOR}"
        else:
            return log_message

LOG_FORMAT = "[%(asctime)s] - [%(levelname)s] - '%(message)s'"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S %p"

logger = logging.getLogger("pavs_logger")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)

formatter = ColoredFormatter(LOG_FORMAT, datefmt=DATE_FORMAT)
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

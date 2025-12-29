from loguru import logger
import sys

def setup_logging():
    logger.remove()  # Remove default handler
    logger.add(
        sys.stderr,
        level="INFO", # Default log level
        format="{time} {level} {message}",
        colorize=True,
        diagnose=False # Disable showing source code in tracebacks
    )
    logger.add(
        "logs/app.log",
        level="DEBUG",
        rotation="10 MB", # Rotate file every 10 MB
        compression="zip", # Compress old log files
        enqueue=True, # Use a separate thread for logging
        diagnose=False # Disable showing source code in tracebacks
    )
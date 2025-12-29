import sys
from loguru import logger

def setup_logging():
    logger.remove()  # Remove default handler
    logger.add(
        sys.stderr,
        level="INFO",
        format="{time} {level} {message}",
        colorize=True,
        backtrace=True,
        diagnose=True,
    )
    # Add a file handler for more persistent logs
    logger.add(
        "logs/app.log",
        rotation="10 MB",  # Rotate file if it exceeds 10 MB
        compression="zip",  # Compress rotated files
        level="INFO",
        format="{time} {level} {message}",
        enqueue=True, # Use a multiprocess-safe queue
    )

    # Optionally, set up different levels for different modules
    # For example, to silence verbose SQLAlchemy logs
    logger.opt(colors=True).info("<green>Logging configured successfully!</green>")

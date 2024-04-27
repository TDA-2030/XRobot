# -*- coding:utf-8 -*-

import logging
import time
import io
from os import path as ospath
from typing import Any
import warnings

__all__ = ("MyLog", "TqdmToLogger")


class _CustomFormatter(logging.Formatter):

    BLUE = "\x1b[32;20m"
    GREY = "\x1b[38;20m"
    YELLOW = "\x1b[33;20m"
    RED = "\x1b[31;20m"
    BOLD_RED = "\x1b[31;1m"
    RESET = "\x1b[0m"

    def __init__(self, fmt, colorful: bool = True):
        self.FORMATS = {
            logging.DEBUG:
            _CustomFormatter.GREY + fmt + _CustomFormatter.RESET,
            logging.INFO:
            _CustomFormatter.BLUE + fmt + _CustomFormatter.RESET,
            logging.WARNING:
            _CustomFormatter.YELLOW + fmt + _CustomFormatter.RESET,
            logging.ERROR:
            _CustomFormatter.RED + fmt + _CustomFormatter.RESET,
            logging.CRITICAL:
            _CustomFormatter.BOLD_RED + fmt + _CustomFormatter.RESET
        }
        self.fmt = fmt
        self.colorful = colorful

    def format(self, record):
        if self.colorful:
            log_fmt = self.FORMATS.get(record.levelno)
        else:
            log_fmt = self.fmt
        formatter = logging.Formatter(log_fmt)
        formatter.default_time_format = '%H:%M:%S'
        formatter.default_msec_format = '%s.%03d'
        return formatter.format(record)


class MyLog():
    _instance = None
    def __new__(cls, *args, **kw):
        if cls._instance is None:
            cls._instance = object.__new__(cls, *args, **kw)
        return cls._instance

    def __init__(self) -> None:
        self.my_logger = logging.getLogger("my_logger")
        self.my_logger.setLevel(logging.DEBUG)
        self._fmt = "%(asctime)s-%(levelname)-8s- %(message)s"
        if not self.my_logger.handlers:
            self._sh1 = logging.StreamHandler()
            self._sh1.setFormatter(_CustomFormatter(self._fmt))
            self.my_logger.addHandler(self._sh1)

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        msg=""
        for m in args:
            msg += str(m)
        self.my_logger.info(msg)

    def set_file_dir(self, log_file_dir: str = None):
        if log_file_dir:
            log_file_dir = ospath.join(
                log_file_dir, "%s.log" %
                time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime()))
            self._fh1 = logging.FileHandler(filename=log_file_dir, mode='w')
            self._fh1.setFormatter(_CustomFormatter(
                self._fmt,
                colorful=False))  # file can't show color correctly, disable it
            self.my_logger.addHandler(self._fh1)
        else:
            try:
                self.my_logger.removeHandler(self._fh1)
            except AttributeError:
                warnings.warn("No file handler has been added")

    def warning(self, msg, *args, **kwargs):
        self.my_logger.warning(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self.my_logger.info(msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        self.my_logger.debug(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.my_logger.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self.my_logger.critical(msg, *args, **kwargs)


class TqdmToLogger(io.StringIO):
    """
        Output stream for TQDM which will output to logger module instead of
        the StdOut.

        example:

        from tqdm import tqdm
        logging.basicConfig(format='%(asctime)s [%(levelname)-8s] %(message)s')
        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)

        tqdm_out = TqdmToLogger(logger)
        for x in tqdm(range(100),file=tqdm_out,mininterval=30,):
            time.sleep(.5)
    """
    logger = None
    level = None
    buf = ''

    def __init__(self, logger: MyLog):
        super(TqdmToLogger, self).__init__()
        self.logger = logger

    def write(self, buf):
        self.buf = buf.strip('\r\n\t ')

    def flush(self):
        self.logger(self.buf)


if __name__ == '__main__':
    # test it
    my_logger = MyLog()
    my_logger("Call instance directly to print info")
    my_logger.debug("This is  DEBUG of my_logger !!")
    my_logger.info("This is  INFO of my_logger !!")
    my_logger.warning("This is  WARNING of my_logger !!")
    my_logger.error("This is  ERROR of my_logger !!")
    my_logger.critical("This is  CRITICAL of my_logger !!")

    my_logger.set_file_dir()  # should trig a warning
    my_logger.set_file_dir("./")
    my_logger.debug("This is  DEBUG of my_logger !!")
    my_logger.info("This is  INFO of my_logger !!")
    my_logger.warning("This is  WARNING of my_logger !!")
    my_logger.error("This is  ERROR of my_logger !!")
    my_logger.set_file_dir(
    )  # remove file, so next message should not appear in the log file
    my_logger.critical("This is  CRITICAL of my_logger !!")

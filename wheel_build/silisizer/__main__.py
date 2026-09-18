import os
import sys

from . import SILISIZER_BIN_PATH


def silisizer():
    os.execl(SILISIZER_BIN_PATH, "silisizer", *sys.argv[1:])


if __name__ == "__main__":
    silisizer()

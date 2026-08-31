import os
import sys

from . import SILISIZER_BIN_PATH, TCLLIB_PATH


def silisizer():
    env = os.environ.copy()
    env["TCL_LIBRARY"] = TCLLIB_PATH
    os.execle(SILISIZER_BIN_PATH, "silisizer", *sys.argv[1:], env)


if __name__ == "__main__":
    silisizer()

##### IMPORTS
import numpy as np

##### SELF-DEFINED IMPORTS

##### CSV FILE IMPORT FUNCTION (imports from filepath and returns np array)
def ImportCSV(filepath: str) -> np.array:
    # returning array from data
    return np.loadtxt(fname=filepath, delimiter=',')
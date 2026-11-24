##### PYDRAKE IMPORTS

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### LOG SAVE FUNCTION (inputs data and col number and saves)
def LogSave(data: np.ndarray, cols: int, filename: str) -> None:
    print(f'\n=====SAVING {filename}=====')

    # input validation
    if np.size(data) == 0: return # return if not data to save
    assert (np.size(data) / cols).is_integer(), 'Incorrect desired data shape, check rows!'

    # reshaping data and saving
    rows = int(np.size(data) / cols)
    data = np.reshape(data, (-1, rows)).T
    
    np.save(str(config.DAT_DIR / filename), data)
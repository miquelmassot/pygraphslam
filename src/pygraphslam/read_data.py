import numpy as np


def read_data(filename):
    # Read Data
    lasers = []
    odoms = []
    with open(filename) as f:
        for line in f:
            tokens = line.split(" ")
            if tokens[0] == "FLASER":
                num_readings = int(tokens[1])
                scans = np.array(tokens[2 : 2 + num_readings], dtype=np.float)
                index = np.arange(-90, 90 + 180 / num_readings, 180 / num_readings)
                index = np.delete(index, num_readings // 2)
                converted_scans = []
                angles = np.radians(index)
                converted_scans = (
                    np.array([np.cos(angles), np.sin(angles)]).T * scans[:, np.newaxis]
                )
                lasers.append(np.array(converted_scans))
                x = float(tokens[2 + num_readings])
                y = float(tokens[3 + num_readings])
                theta = float(tokens[4 + num_readings])
                odoms.append([x, y, theta])
    return np.array(odoms), np.array(lasers)

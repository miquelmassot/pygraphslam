import unittest
from pathlib import Path

from pygraphslam.read_data import read_data


class TestReadData(unittest.TestCase):
    def test_read_data(self):
        filename = Path(__file__).parent / "data" / "test_data.clf"
        odoms, lasers = read_data(filename)
        self.assertEqual(len(odoms), len(lasers))
        self.assertEqual(len(lasers), 3)

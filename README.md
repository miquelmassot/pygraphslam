# pyGraphSLAM

[![pygraphslam](https://github.com/miquelmassot/pygraphslam/actions/workflows/pygraphslam.yaml/badge.svg)](https://github.com/miquelmassot/pygraphslam/actions/workflows/pygraphslam.yaml)
[![codecov](https://codecov.io/gh/miquelmassot/pygraphslam/branch/main/graph/badge.svg?token=wAR4eTc3uT)](https://codecov.io/gh/miquelmassot/pygraphslam)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Say Thanks!](https://img.shields.io/badge/Say%20Thanks-!-1EAEDB.svg)](https://saythanks.io/to/miquelmassot)

A python implementation of SLAM using laser scan matching ICP frontend and a g2o backend using [g2opy](https://github.com/miquelmassot/g2opy).

![GIF](https://github.com/miquelmassot/pygraphslam/raw/main/data/pygraphslam.gif)

## Usage
```
usage: pygraphslam [-h] --input INPUT [--draw_last DRAW_LAST] [--save_gif]

Python Graph Slam

optional arguments:
  -h, --help            show this help message and exit
  --input INPUT         Input CLF File.
  --draw_last DRAW_LAST
                        Number of point clouds to draw.
  --save_gif            Save a GIF animation
```

## License
Based on https://github.com/goktug97/PyGraphSLAM with MIT license.

## Datasets
http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php

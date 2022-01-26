#!/usr/bin/env python

import argparse
from pygraphslam.graph_slam import GraphSlam
from pygraphslam.read_data import read_data

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Python Graph Slam')
    parser.add_argument('--input', type=str, help='Input CLF File.', required=True)
    parser.add_argument('--draw_last', default=float('inf'), type=int,
                        help='Number of point clouds to draw.')
    parser.add_argument('--save_gif', dest='save_gif', action='store_true')
    parser.set_defaults(save_gif=False)
    args = parser.parse_args()

    slam = GraphSlam(args.save_gif, args.draw_last)

    odoms, lasers = read_data(args.input)

    for odom_idx, odom in enumerate(odoms):
        slam.iterate(odom, lasers[odom_idx])
        if odom_idx == args.draw_last:
            break

    """
    for odom_idx, odom in enumerate(odoms):
        # Initialize
        if odom_idx == 0:
            prev_odom = odom.copy()
            prev_idx = 0
            B = lasers[odom_idx]
            registered_lasers.append(B)
            continue
    """

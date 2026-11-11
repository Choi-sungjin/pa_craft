#!/usr/bin/env python3
"""Loop closures without RViz: SLAM map + pose graph from a rosbag, drawn to a PNG.

Record during a mapping run (topics remapped by craft.launch.py):
  ros2 bag record -o <bag> --topics /perception/lidar/map /perception/lidar/slam_toolbox/pose_graph \\
      /perception/lidar/slam_toolbox/loop_closure_event /perception/lidar/slam_toolbox/new_node_event
Draws the last map, the graph nodes and edges; an edge between nodes more than --gap ids apart is a loop
closure and is drawn thick. Scan matching links each node to the last scan_buffer_size (slam.yaml: 100)
nodes, so the default gap is 100; revisits that join older nodes (loop closures and near chains) count. Prints the loop closure events with
their time. Without a pose graph in the bag, the nodes come from the new-node events (no closures drawn).
Live check over ssh instead: ros2 topic echo /perception/lidar/slam_toolbox/loop_closure_event

  slam_graph.py <bag dir> [--gap 100]
"""
import argparse
import sys
import warnings
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

warnings.filterwarnings('ignore', message='Use read_next_ext', category=DeprecationWarning)  # stamps unused
PREFIX = '/perception/lidar/'
MAP, GRAPH = PREFIX + 'map', PREFIX + 'slam_toolbox/pose_graph'
CLOSURE, NODE = PREFIX + 'slam_toolbox/loop_closure_event', PREFIX + 'slam_toolbox/new_node_event'


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    last, closures, nodes, t0 = {}, [], {}, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        t0 = t if t0 is None else t0
        if topic not in (MAP, GRAPH, CLOSURE, NODE):
            continue
        m = deserialize_message(data, types[topic])
        if topic == CLOSURE:
            closures.append((t - t0) * 1e-9)
        elif topic == NODE:
            nodes[m.new_node_id] = (m.pose.position.x, m.pose.position.y)
        else:
            last[topic] = m
    return last.get(MAP), last.get(GRAPH), closures, nodes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', type=Path)
    ap.add_argument('--gap', type=int, default=100,
                    help='edges spanning more node ids are loop closures (slam.yaml scan_buffer_size)')
    args = ap.parse_args()
    grid, graph, closures, node_events = read(args.bag)
    if grid is None and graph is None and not node_events:
        sys.exit(f'{args.bag}: no map, pose graph or node events')

    if graph is not None:
        nodes = {n.node_id: (n.pose.position.x, n.pose.position.y) for n in graph.nodes}
        edges = [(e.source_id, e.target_id) for e in graph.edges if e.source_id in nodes and e.target_id in nodes]
    else:
        nodes, edges = node_events, []
    loops = [(a, b) for a, b in edges if abs(a - b) > args.gap]
    print(f'[graph] {len(nodes)} nodes, {len(edges)} edges, {len(loops)} loop closure edges '
          f'(> {args.gap} ids apart){"" if graph is not None else " - no pose graph in the bag"}')
    for a, b in sorted(loops, key=lambda e: -abs(e[0] - e[1]))[:10]:  # the widest revisits
        (xa, ya), (xb, yb) = nodes[a], nodes[b]
        print(f'[graph]   node {min(a, b)} <-> {max(a, b)}: {abs(a - b)} nodes later, '
              f'{np.hypot(xa - xb, ya - yb):.2f} m apart in the map')
    if len(loops) > 10:
        print(f'[graph]   ... {len(loops) - 10} more')
    print(f'[graph] loop closure events: {len(closures)}'
          + (' at ' + ', '.join(f'{t:.0f} s' for t in closures) if closures else ''))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 9), dpi=130)
    fig.patch.set_facecolor('#fcfcfb'); ax.set_facecolor('#fcfcfb')
    if grid is not None:
        i = grid.info
        cells = np.array(grid.data, dtype=np.int16).reshape(i.height, i.width)
        shade = np.where(cells < 0, 0.93, 1.0 - np.clip(cells, 0, 100) / 100.0 * 0.85)  # unknown light gray
        ax.imshow(shade, cmap='gray', vmin=0, vmax=1, origin='lower', interpolation='nearest',
                  extent=[i.origin.position.x, i.origin.position.x + i.width * i.resolution,
                          i.origin.position.y, i.origin.position.y + i.height * i.resolution])
    for a, b in edges:
        if (a, b) not in loops:
            ax.plot(*zip(nodes[a], nodes[b]), color='#2a78d6', lw=0.8, alpha=0.6)
    for k, (a, b) in enumerate(loops):
        ax.plot(*zip(nodes[a], nodes[b]), color='#eb6834', lw=2.5, label='loop closure' if k == 0 else None)
    if nodes:
        xy = np.array([nodes[k] for k in sorted(nodes)])
        ax.plot(xy[:, 0], xy[:, 1], 'o', color='#2a78d6', ms=2.5, label=f'graph nodes ({len(nodes)})')
        ax.plot(*xy[0], marker='s', color='#0b0b0b', ms=8, ls='none', label='first node')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (map)', color='#0b0b0b'); ax.set_ylabel('y [m]', color='#0b0b0b')
    ax.set_title(f'SLAM pose graph ({args.bag.name}): {len(loops)} loop closure edges, '
                 f'{len(closures)} events', color='#0b0b0b', fontsize=11, loc='left')
    ax.tick_params(colors='#52514e')
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, fontsize=9, loc='best')
    out = args.bag.parent / f'{args.bag.name}_graph.png'
    fig.tight_layout(); fig.savefig(out)
    print(f'[graph] plot {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

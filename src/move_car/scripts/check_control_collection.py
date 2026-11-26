#!/usr/bin/env python3
"""Read-only master graph/parameter preflight. No node start or ROS publisher."""
import argparse
import json
import os
from pathlib import Path
from xmlrpc.client import ServerProxy, Transport, Error
from control_research.collection_contract import collection_preflight
from control_research.dataset import write_new


class BoundedTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def read_master(uri, s8_node):
    master = ServerProxy(uri, transport=BoundedTransport())
    caller = '/p42_read_only_collection_preflight'
    def call(method, *args):
        code, message, value = getattr(master, method)(caller, *args)
        if code != 1:
            raise ValueError(message)
        return value
    def param(name, default):
        code, _, value = master.getParam(caller, name)
        return value if code == 1 else default
    graph = call('getSystemState')
    return dict(publishers=graph[0], subscribers=graph[1],
                published_types=call('getPublishedTopics', ''),
                parameters={s8_node: param(s8_node, {}),
                            '/trajectory_reconstructor': param('/trajectory_reconstructor', {})},
                run_id=param('/control_research/run_id', None), use_sim_time=param('/use_sim_time', False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, help='offline graph snapshot; no ROS connection')
    parser.add_argument('--master-uri', default=os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
    parser.add_argument('--expected-run-id', required=True)
    parser.add_argument('--s8-node', default='/publish_speed_turn')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        snapshot = json.loads(args.snapshot.read_text()) if args.snapshot else read_master(args.master_uri, args.s8_node)
        result = collection_preflight(snapshot, args.expected_run_id, args.s8_node)
    except (OSError, ValueError, Error) as error:
        result = collection_preflight({}, args.expected_run_id, args.s8_node)
        result['missing'].append('graph_or_snapshot_unavailable:' + str(error))
    result['snapshot_source'] = str(args.snapshot) if args.snapshot else 'read_only_ROS_master_RPC'
    write_new(args.output, result)
    print('local_tracking_topics_present' if result.get('local_tracking_topics_ready') else
          'local_tracking_topics_incomplete')
    print('extended_research_snapshot_present' if result['full_chain_topics_ready'] else
          'extended_research_snapshot_incomplete; this is not a verdict on S8/local tracking')


if __name__ == '__main__':
    main()

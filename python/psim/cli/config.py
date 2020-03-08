from .. import config

import argparse
import json


def main(args):
    
    parser = argparse.ArgumentParser(
        prog='psim config', description='Process PSim configuration files.'
    )
    parser.add_argument(
        '--modules', '-m', nargs='+', required=False, type=str, help='Set of ' +
        'modules to process configuration files for. Defaults to all modules ' +
        'none are specified.'
    )
    parser.add_argument(
        '--output', '-o', type=argparse.FileType('w'), default='-',
        help='Destination file for the processed configuration file. Defaults' +
        ' to the stdout.'
    )
    parser.add_argument(
        '--path', '-p', required=True, type=str, help='PSim configuration directory.'
    )
    args = parser.parse_args(args)

    leader, follower = config.load_module(args.modules[0], args.path)

    dictionary = {
      "leader": leader,
      "follower": follower
    }
    json.dump(dictionary, args.output, indent=4, sort_keys=True)

    return 0

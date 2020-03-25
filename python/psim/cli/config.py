from .. import config

import json
import logging
import sys

log = logging.getLogger(__name__)

prog='psim config'
desc='Process PSim configuration files.'


def arguments(parser):
    parser.add_argument(
        '--config-dir', '-c', required=False, type=str, default='./', help='' +
        'Configuration directory for the simulation. This is a relative path ' +
        'from "psim/config" in the PSim repository and default to "./".'
    )
    parser.add_argument(
        '--patch-dir', '-p', required=False, type=str, help='Looks for an ' +
        'extra set of configuration files in the patch directory. This ' +
        'search is not recursive and is primarily intended to remove the ' +
        'effects on a Monte Carlo configuration.'
    )
    parser.add_argument(
        '--output-dir', '-o', required=False, type=str, default='./', help='' +
        'Output directory to dump the processed configuration files in. This ' +
        'defaults to your current working directory.'
    )
    parser.add_argument(
        '--modules', '-m', required=False, type=str, help='Comma separated ' +
        'list of modules to process configuration files for. Defaults to all ' +
        'modules not specified.'
    )
    return parser


def main(args):
    # Process the modules list if specified otherwise default to process all
    # modules
    if args.modules:
        modules = set(args.modules.split(','))
        for module in modules:
            if module not in config.all_modules:
                print('ERROR: "%s" is not a valid module' % module, file=sys.stderr)
                return -1
    else:
        modules = set(config.all_modules)

    # leader, follower = config.load_module(args.modules[0], args.path)

    # dictionary = {
    #   "leader": leader,
    #   "follower": follower
    # }
    # json.dump(dictionary, args.output, indent=4, sort_keys=True)

    return 0

from .. import config

import json
import logging
import os
import sys

log = logging.getLogger(__name__)

prog='psim config'
desc='Process PSim configuration files.'


def arguments(parser):
    parser.add_argument(
        'config_dir', metavar='CONFIG_DIR', type=str, help='Configuration ' +
        'directory for the simulation. This is a relative path from ' +
        '"python/psim/config" in the PSim repository.'
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
    parser.add_argument(
        '--readable', '-r', action='store_true', help='Enable cleaner JSON ' +
        'formatting for the resulting output files.'
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

    # Load configuration data
    data = config.load_modules(modules, args.config_dir, args.patch_dir)

    # Handle readable output if requested
    json_kwargs = dict()
    if args.readable:
        json_kwargs.update({'indent': 4, 'sort_keys': True})

    # Dump data to the proper JSON files
    for module in modules:
        output_file = os.path.abspath(os.path.expanduser(os.path.join(args.output_dir, '{}.json'.format(module))))
        with open(output_file, 'w') as ostream:
            json.dump(data[module], ostream, **json_kwargs)

    return 0

from . import cli

import argparse
import logging
import sys

command_map = {
    "config": cli.config
}


def print_usage():
    print('USAGE: python -m psim [command] ...', file=sys.stderr)
    print('Valid commands include:', list(command_map.keys()), file=sys.stderr)


def main():
    # Ensure a valid command was specified
    if len(sys.argv) < 2:
        print('ERROR: No command specified', file=sys.stderr)
        print_usage()
        return -1
    if sys.argv[1] not in command_map.keys():
        print('ERROR: "%s" is an invalid command' % sys.argv[1], file=sys.stderr)
        print_usage()
        return -1

    # Grab the module and build the command parser
    module = command_map[sys.argv[1]]
    parser = argparse.ArgumentParser(prog=module.prog, description=module.desc)
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable ' +
        'verbose, debugging output.'
    )
    parser.add_argument('-q', '--quiet', action='store_true', help='Disable ' +
        'all logging output except for fatal errors. This will disable the ' +
        'verbose flag.'
    )
    parser = module.arguments(parser)
    args = parser.parse_args(sys.argv[2:])

    # Setup the logging library
    logging_level = logging.INFO
    if args.verbose:
        logging_level = logging.DEBUG
    if args.quiet:
        logging_level = logging.ERROR
    logging.basicConfig(format='[%(levelname)s %(name)s]: %(message)s', level=logging_level)

    # Call the module's function
    return module.main(args)


if __name__ == '__main__':
    sys.exit(main())

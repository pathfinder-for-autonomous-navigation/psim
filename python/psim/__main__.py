from . import cli as _cli

import sys

_command_map = {
    "config": _cli.config
}


def _print_usage():
    print('USAGE: python -m psim [cmd] ...', file=sys.stderr)
    print('Valid commands include:', list(_command_map.keys()), file=sys.stderr)


def main():
    
    if len(sys.argv) < 2:
        print('ERROR: No command specified', file=sys.stderr)
        _print_usage()
        return -1

    if sys.argv[1] not in _command_map.keys():
        print('ERROR: "%s" is an invalid command' % sys.argv[1], file=sys.stderr)
        _print_usage()
        return -1

    return _command_map[sys.argv[1]].main(sys.argv[2:])


if __name__ == '__main__':
    sys.exit(main())

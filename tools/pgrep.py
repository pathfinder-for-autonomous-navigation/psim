#!/usr/bin/env python3

"""Simple command line utility to quickly search through state field names
declared by psim models.
"""

from dataclasses import dataclass

import argparse
import glob
import re
import sys
import textwrap
import yaml


@dataclass
class Field:
    """Keeps track of data related to a field being added to the simulation.
    """
    name: str
    specifier: str
    comment: str
    source: str


def get_printer(verbose=False):
    """Returns a printer function for logging field information. The function
    returned varies on the verbosity requested.
    """
    wrapper = textwrap.TextWrapper(
        width=80, initial_indent='\t\t', subsequent_indent='\t'
    )

    def brief_print(field: Field):
        print(f'{field.name} [{field.specifier}]')

    def verbose_print(field: Field):
        print(field.name, ':')
        print('\ttype:', field.specifier)
        print('\tsource:', field.source)
        print('\tcomment:')
        for line in wrapper.wrap(field.comment.rstrip()):
            print(line)

    return verbose_print if verbose else brief_print


def main() -> int:
    # Specify and parse the command line arguments
    parser = argparse.ArgumentParser(
        description='Simple command line tool to search for state fields ' +
        'declared within the psim repository. Note that the regular ' +
        'expressions use Python\'s syntax.'
    )
    parser.add_argument(
        '-v', '--verbose', action='store_true', help='increases the amount ' +
        'of information output to the terminal'
    )
    parser.add_argument(
        'regex', metavar='REGEX', type=str, nargs='+', help='Set of regular ' +
        'expressions to search for field names with'
    )
    args = parser.parse_args()

    # Collect all the state fields from the model interfaces
    fields = list()
    for model in glob.glob('include/**/*.yml', recursive=True):
        try:
            with open(model, 'r') as istream:
                data = yaml.safe_load(istream)
        except e:
            print(
                f'[ERROR] Error while parsing "{model}"; continuing.',
                file=sys.stderr
            )
            continue

        if type(data) is not dict:
            print(
                f'[ERROR] YAML given by "{model}" does not provide a ' +
                'dictionary; continuing.', file=sys.stderr
            )
            continue

        for field in data.get('adds', list()):
            field['specifier'] = field.get('type', '')
            del field['type']

            fields.append(Field(source=model, **field))

    # Search for potential matches per regular expression and output results
    printer = get_printer(args.verbose)
    for regex in args.regex:
        print(f'[INFO] Searching now with "{regex}" ...')

        program = re.compile(regex)
        for field in fields:
            if program.match(field.name):
                printer(field)

        print(f'[INFO] Complete!')

    return 0


if __name__ == '__main__':
    sys.exit(main())

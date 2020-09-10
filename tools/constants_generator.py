import csv

CSV_FILE='tools/constants.csv'

HPP_FILE='include/gnc/constants.hpp'
HPP_HEADER='''\
/** @file gnc/constants.hpp
 *  Autocoded constants header file. See tools/constants_generator.py for more
 *  information. */

#ifndef GNC_CONSTANTS_HPP_
#define GNC_CONSTANTS_HPP_

#include "config.hpp"

#include <lin/core.hpp>

#include <cstdint>

#include <limits>

namespace gnc {
namespace constant {

'''
HPP_FOOTER='''\
}  // namespace constant
}  // namespace gnc

#endif

'''

CPP_FILE='src/gnc_constants.cpp'
CPP_HEADER='''\
/** @file gnc_constants.cpp
 *  Autocoded constants source file. See tools/constants_generator.py for more
 *  information. */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>

namespace gnc {
namespace constant {

'''
CPP_FOOTER='''\
}  // namespace constant
}  // namespace gnc

'''


def generate(csv_file, hpp_file, cpp_file):
    '''This function generates a constants header and source file for the PSim
    CXX library given a constants CSV file. This script is automatically run
    prior to each PSim PlatformIO build.

    The format of the CSV file is as follows:

        <editable>,<type>,<name>,<value 0>,<value 1>,...

    - <editable> specifies whether or not the constant will be treated as an
      extern (i.e. can be edited by flight software) and is either "true" or
      "false".
    - <type> specifies the type of the constant.
    - <name> is the name of the constant in PSim CXX source code.
    - <value 0>,... is a variadic static initializer list which sets the
      constant's initial value.
    '''
    hpp_file.write(HPP_HEADER)
    cpp_file.write(CPP_HEADER)

    # Loop over each constant entry
    for constant in csv.reader(csv_file, delimiter=','):
        # Ensure at least four entries
        if len(constant) < 4:
            print('ERROR: CSV line with less than four elements detected')
            continue

        # External value
        if constant[0].lower() == 'true':
            hpp_file.write('extern {0} {1};\n\n'.format(constant[1], constant[2]))
            cpp_file.write('GNC_TRACKED_CONSTANT({0}, {1}, {2}'.format(constant[1], constant[2], constant[3]))
            for arg in constant[4:]:
                cpp_file.write(', {}'.format(arg))
            cpp_file.write(');\n\n')

        # Constexpr value
        elif constant[0].lower() == 'false':
            hpp_file.write('GNC_TRACKED_CONSTANT(constexpr static {0}, {1}, {2}'.format(constant[1], constant[2], constant[3]))
            for arg in constant[4:]:
                hpp_file.write(', {}'.format(arg))
            hpp_file.write(');\n\n')

    hpp_file.write(HPP_FOOTER)
    cpp_file.write(CPP_FOOTER)


with open(CSV_FILE, 'r') as csv_file:
    with open(HPP_FILE, 'w') as hpp_file:
        with open(CPP_FILE, 'w') as cpp_file:
            generate(csv_file, hpp_file, cpp_file)

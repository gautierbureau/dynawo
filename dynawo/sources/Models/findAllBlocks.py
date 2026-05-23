import os
import re
import sys

# A component instantiation declares an instance of a qualified class:
#   "<Upper.Qualified.Type> <lowercaseInstanceName> ...".
# Class paths are UpperCamelCase per segment while instance names are
# lowerCamelCase; that convention separates instantiations from
# parameter/variable declarations (named UpperCamelCase in the Dynawo
# library) and from comments and equations.
_INSTANCE_RE = re.compile(r'([A-Z]\w*(?:\.[A-Z]\w*)+)\s+[a-z]\w*')

# Qualified paths that are unit/constant/icon definitions, not blocks.
_NON_BLOCK_SEGMENTS = ('Types', 'SIunits', 'Units', 'Constants', 'Icons')


def find_mo_files(directory):
    mo_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.mo'):
                if 'package' not in file and 'Examples' not in root and 'VariableImpedantFault' not in file:
                    mo_files.append(os.path.join(root, file))
    return mo_files


def _is_block_path(path):
    return not any(segment in _NON_BLOCK_SEGMENTS for segment in path.split('.'))


def find_blocks_in_file(mo_file):
    blocks = set()
    with open(mo_file, 'r', errors='ignore') as file:
        for line in file:
            match = _INSTANCE_RE.match(line.strip())
            if match and _is_block_path(match.group(1)):
                blocks.add(match.group(1))
    return blocks


def findAllBlocks(directory):
    blocks = set()
    for mo_file in find_mo_files(directory):
        blocks |= find_blocks_in_file(mo_file)
    for block in sorted(blocks):
        print(block)


if __name__ == "__main__":
    findAllBlocks(sys.argv[1])

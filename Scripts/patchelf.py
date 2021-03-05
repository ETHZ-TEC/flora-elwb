#!/usr/bin/python3
#
# Binary patching using pyelftools.
#
# Inspired by:
# * http://danielpocock.com/elfpatch
# * https://github.com/eliben/pyelftools/issues/227#issuecomment-495184612
#
#
################################################################################

import os
import sys
import struct

# pyelftools from https://github.com/eliben/pyelftools
from elftools.elf.elffile import ELFFile
from elftools.elf.elffile import Section
from elftools.elf.sections import SymbolTableSection

################################################################################

def structSizeMap(numBytes):
    if numBytes == 1: return 'B'
    elif numBytes == 2: return 'H'
    elif numBytes == 4: return 'I'
    elif numBytes == 8: return 'Q'
    else: raise Exception('Undefined number of bytes!')

def getSymbolFileOffsetAndSize(elfPath, symbName):
    '''Get the file offset (in bytes) and the size of the symbol with name symbName from ELF file elfPath.
    '''
    fileOffset = None
    symbSize = None

    with open(elfPath, 'rb') as f:
        elf = ELFFile(f)

        # read symbol table
        symtab = elf.get_section_by_name('.symtab')
        if not symtab:
            raise Exception('ELF file does not contain a symbol table!')
        if not isinstance(symtab, SymbolTableSection):
            raise Exception('Not a valid symbol table!')

        # get symbol & symbol size
        symList = symtab.get_symbol_by_name(symbName)
        if not symList:
            raise Exception('Symbol "{}" not found!'.format(symbName))
        sym = symList[0]
        symbSize = sym['st_size']

        # find segment with symbol
        for seg in elf.iter_segments():
            if seg.header['p_type'] != 'PT_LOAD':
                continue

            if (sym['st_value'] >= seg['p_vaddr']) and (sym['st_value'] < seg['p_vaddr'] + seg['p_filesz']):
                # get file offset
                fileOffset = sym['st_value'] - seg['p_vaddr'] + seg['p_offset']
                break

        if not fileOffset:
            raise Exception('Could not determine file offset!')

    return fileOffset, symbSize

def readSymbolValue(elfPath, symbName, signed=False):
    '''Read value of symbol symbName from elf file elfPath.
    '''
    value = None

    with open(elfPath, 'rb') as f:
        elf = ELFFile(f)
        fileOffset, symbSize = getSymbolFileOffsetAndSize(elfPath, symbName)
        elf.stream.seek(fileOffset)
        if signed:
            value = struct.unpack(structSizeMap(symbSize).lower(), elf.stream.read(symbSize))[0]
        else:
            value = struct.unpack(structSizeMap(symbSize), elf.stream.read(symbSize))[0]

    return value

def writeSymbolValue(elfPath, symbName, symbReplace, signed=False):
    '''Searches symbol symbName in ELF file elfPath and replaces its value with symbReplace.
    Args:
        elfPath:        Path to the elf file in which the symbol will be replaced
        symbName:       Name of the symbol whose value will be replaced
        symbReplace:    Replacement value (int)
        signed:         If True, the symbol is assumed to be a signed integer, otherwise the symbol is assumed to be an unsigned integer (default: False)
    '''
    symbFileOffset, symbSize = getSymbolFileOffsetAndSize(elfPath, symbName)

    with open(elfPath, 'r+b') as f:
        f.seek(symbFileOffset)
        if signed:
            replaceBytes = struct.pack(structSizeMap(symbSize).lower(), symbReplace)
        else:
            replaceBytes = struct.pack(structSizeMap(symbSize), symbReplace)
        f.write(replaceBytes)
        f.close()


if __name__ == '__main__':
    elfPath = './Debug/comboard_elwb.elf'
    symbName = 'gloria_modulation'
    symbReplace = 10
    signed = True

    symbValue = readSymbolValue(elfPath, symbName, signed=signed)
    print('Value BEFORE patching of symbol "{}": {}'.format(symbName, symbValue))

    writeSymbolValue(elfPath, symbName, symbReplace, signed=signed)

    symbValue = readSymbolValue(elfPath, symbName, signed=signed)
    print('Value AFTER patching of symbol "{}": {}'.format(symbName, symbValue))

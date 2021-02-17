#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: rtrueb
@brief: Creates and executes flocklab tests
"""

import re
import os
import datetime
from elftools.elf.elffile import ELFFile
import json
import git

from flocklab import Flocklab
from flocklab import *

fl = Flocklab()


###############################################################################
# FL2
obsNormal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32]
obsHg = []

DURATION = 70
imageNormalId = 'imageNormal'
imageHgId = 'imageHg'
cwd = os.path.dirname(os.path.realpath(__file__))
imagePath = os.path.join(cwd, '../Debug/comboard_elwb.elf')
xmlPath = os.path.join(cwd, 'flocklab_dpp2lora_elwb.xml')
obsList = obsNormal + obsHg

###############################################################################

def readConfig(symbol, configFile='../Inc/app_config.h', idx=0):
    with open(os.path.join(cwd, configFile), 'r') as f:
        text = f.read()

    # ret = re.search(r'#define\s+({})\s+(.+?)\s'.format(symbol), text)
    ret = re.findall(r'#define\s+({})\s+(.+?)\s'.format(symbol), text)
    if len(ret) == 0:
        raise Exception('ERROR: readConfig: element "{}" not found'.format(symbol))
    if idx >= len(ret):
        raise Exception('ERROR: readConfig: idx ({}) out of range'.format(idx))
    ret = ret[idx][1]
    if len(ret) >= 2 and ret[0] == '"' and ret[-1] == '"':
        ret = ret[1:-1]
    if ret.isnumeric():
        ret = int(ret)
    return ret

def get_globalvar_addr(varname, filename='../Debug/comboard_elwb.elf'):
    with open(os.path.join(cwd, filename), 'rb') as f:
        elffile = ELFFile(f)
        symtab = elffile.get_section_by_name('.symtab')
        sym = symtab.get_symbol_by_name(varname)

        if sym is None or len(sym) == 0:
            return None
        elif len(sym) > 1:
            print('WARNNG: multiple entries found!')
        return sym[0]['st_value']

def check_globalvar_exists(dataTraceConfList):
    for elem in dataTraceConfList:
        var = elem[0]
        if not var[0:2] == '0x': # ignore variables directly specified by an address
            if get_globalvar_addr(var) is None:
                raise Exception('ERROR: datatrace variable "{}" is not contained in binary image!'.format(var))

################################################################################
# Test creation
################################################################################

def create_test():
    # read info from config/header files
    custom = dict()
    for var in ['FLOCKLAB', 'HOST_ID', 'FLOCKLAB_SWD', 'SWO_ENABLE']:
        custom[var] = readConfig(var)
    for var in ['GIT_REV', 'BUILD_TIME']:
        custom[var] = readConfig(var, configFile='../Inc/gitrev.h')
    custom['git_hashes'] = {
        'comboard_elwb': git.Repo(os.path.join(cwd, '..')).head.object.hexsha,
        'flora-lib': git.Repo(os.path.join(cwd, '../Lib')).head.object.hexsha,
        'dpp': git.Repo(os.path.join(cwd, '../Lib/dpp')).head.object.hexsha,
    }

    # sanity check
    if custom['FLOCKLAB'] != 1:
        raise Exception('FLOCKLAB not set to 1 in "app_config.h"!')

    fc = FlocklabXmlConfig()
    fc.generalConf.name = 'eLWB data collection'
    fc.generalConf.description = ''
    # fc.generalConf.startTime = datetime.datetime(2020, 4, 28, 8, 0)
    fc.generalConf.duration = DURATION

    if obsNormal:
        targetNormal = TargetConf()
        targetNormal.obsIds = obsNormal
        targetNormal.embeddedImageId = imageNormalId
        fc.configList.append(targetNormal)

    if obsHg:
        targetHg = TargetConf()
        targetHg.obsIds = obsHg
        targetHg.embeddedImageId = imageHgId
        fc.configList.append(targetHg)

    serial = SerialConf()
    serial.obsIds = obsNormal + obsHg
    serial.port = 'serial'
    serial.baudrate = '460800'
    # serial.remoteIp = '0.0.0.0'
    fc.configList.append(serial)

    gpioTracingConf = GpioTracingConf()
    gpioTracingConf.obsIds = obsList
    pinList = set(['INT1', 'INT2', 'LED1', 'LED2', 'LED3'])
    if custom['FLOCKLAB_SWD']:
        pinList = pinList.difference(set(['INT2', 'LED3']))
    if custom['SWO_ENABLE']:
        pinList = pinList.difference(set(['LED2']))
    gpioTracingConf.pinList = sorted(list(pinList))
    fc.configList.append(gpioTracingConf)

    # gpioActuation = GpioActuationConf()
    # gpioActuation.obsIds = [10]
    # pinConfList = []
    # pinConfList += [{'pin': 'SIG1', 'level': 1, 'offset': 10.0}]
    # pinConfList += [{'pin': 'SIG1', 'level': 0, 'offset': 12.0}]
    # pinConfList += [{'pin': 'SIG1', 'level': 1, 'offset': 15.0}]
    # pinConfList += [{'pin': 'SIG1', 'level': 0, 'offset': 20.0}]
    # pinConfList += [{'pin': 'SIG2', 'level': 1, 'offset': 15.0}]
    # pinConfList += [{'pin': 'SIG2', 'level': 0, 'offset': 20.0}]
    # gpioActuation.pinConfList = pinConfList
    # fc.configList.append(gpioActuation)

    # powerProfiling = PowerProfilingConf()
    # powerProfiling.obsIds = obsList
    # powerProfiling.duration = 30
    # powerProfiling.samplingRate = 1000
    # powerProfiling.fileFormat = 'rld'
    # fc.configList.append(powerProfiling)

    # debugConf = DebugConf()
    # debugConf.obsIds = obsList
    # debugConf.cpuSpeed = 48000000
    # # debugConf.gdbPort = '2331'
    # debugConf.dataTraceConfList = [
    #     # ('0x20006860', 'RW PC'), # xStaticQueue_tx.dummy1 (current length of tx queue)
    #     # ('health_msg_period', 'RW'),
    #     # ('dummy1', 'W PC'),
    #     # ('dummy2', 'R PC'),
    #     # ('dummy3', 'RW PC'),
    #     ('sched_state', 'W', 4),
    # ]
    # fc.configList.append(debugConf)

    if obsNormal:
        imageNormal = EmbeddedImageConf()
        imageNormal.embeddedImageId = imageNormalId
        imageNormal.name = 'dummy_image_name'
        imageNormal.description = 'dummy image description'
        imageNormal.platform = 'dpp2lora'
        imageNormal.imagePath = imagePath
        fc.configList.append(imageNormal)

    if obsHg:
        imageHg = EmbeddedImageConf()
        imageHg.embeddedImageId = imageHgId
        imageHg.name = 'dummy_image_name'
        imageHg.description = 'dummy image description'
        imageHg.platform = 'dpp2lorahg'
        imageHg.imagePath = ''
        fc.configList.append(imageHg)

    fc.generalConf.custom = json.dumps(custom, separators=(',', ':'))
    fc.generateXml(xmlPath=xmlPath)

def run_test():
    # Prompt for scheduling the flocklab test
    inp = input("Submit FlockLab test to FlockLab Server? [y/N]: ")
    if inp == 'y':
        print(fl.createTestWithInfo(xmlPath))
    else:
        print('Test NOT submitted!')


if __name__ == "__main__":
    create_test()
    run_test()

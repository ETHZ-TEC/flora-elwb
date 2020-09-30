#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: rtrueb
@brief: Creates and executes flocklab tests
"""

import re
import os
import datetime

from flocklab import Flocklab
from flocklab import *

fl = Flocklab()


###############################################################################
# FL2
obsNormal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
# obsNormal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 17, 25]
obsHg = []

imageNormalId = 'imageNormal'
imageHgId = 'imageHg'
imagePath = 'Debug/comboard_elwb.elf'
cwd = os.path.dirname(os.path.realpath(__file__))
xmlPath = os.path.join(cwd, 'flocklab_dpp2lora_elwb.xml')
obsList = obsNormal + obsHg

###############################################################################

def readConfig(symbol, configFile='Inc/app_config.h'):
    with open(os.path.join(cwd, configFile), 'r') as f:
        text = f.read()

    ret = re.search(r'#define\s+({})\s+(.+?)\s'.format(symbol), text)
    return ret.groups(0)[1]

def create_test():
    FLOCKLAB = int(readConfig('FLOCKLAB'))
    FLOCKLAB_SWD = int(readConfig('FLOCKLAB_SWD'))
    SWO_ENABLE = int(readConfig('SWO_ENABLE'))
    if FLOCKLAB != 1:
        raise Exception('FLOCKLAB not set to 1 in "app_config.h"!')

    fc = FlocklabXmlConfig()
    fc.generalConf.name = 'eLWB Test'
    fc.generalConf.description = ''
    # fc.generalConf.startTime = datetime.datetime(2020, 4, 28, 8, 0)
    fc.generalConf.duration = 1.5*60

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
    serial.baudrate = '1000000'
    serial.remoteIp = '0.0.0.0'
    fc.configList.append(serial)

    gpioTracingConf = GpioTracingConf()
    gpioTracingConf.obsIds = obsList
    if FLOCKLAB_SWD and SWO_ENABLE:
        gpioTracingConf.pinList = ['INT1', 'LED1']
    elif FLOCKLAB_SWD:
        gpioTracingConf.pinList = ['INT1', 'LED1', 'LED2']
    else:
        gpioTracingConf.pinList = ['INT1', 'INT2', 'LED1', 'LED2', 'LED3']
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
    #     ('0x20006860', 'RW PC'), # xStaticQueue_tx.dummy1 (current length of tx queue)
    #     # ('dummy1', 'W PC'),
    #     # ('dummy2', 'R PC'),
    #     # ('dummy3', 'RW PC'),
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

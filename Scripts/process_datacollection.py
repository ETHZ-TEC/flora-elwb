#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright (c) 2021, ETH Zurich, Computer Engineering Group (TEC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


@author: romantrueb
@brief:  Process data collected by flooding using eLWB
"""

import sys
import os
import numpy as np
import pandas as pd
import json
from collections import OrderedDict
import pickle
import re
import hashlib
import tarfile

from flocklab import Flocklab
from flocklab import *

fl = Flocklab()



################################################################################


FSK_MODULATIONS = [8, 9, 10]

imageConfigToMacro = {
    'host_id': 'HOST_ID',
    'tx_power': 'GLORIA_INTERFACE_POWER',
    'modulation': 'GLORIA_INTERFACE_MODULATION',
    'rf_band': 'GLORIA_INTERFACE_RF_BAND',
    'n_tx': 'ELWB_CONF_N_TX',
    'num_hops': 'ELWB_CONF_NUM_HOPS',
}

imageConfigToGlobalVar = {
    'host_id': 'host_id',
    'tx_power': 'gloria_power',
    'modulation': 'gloria_modulation',
    'rf_band': 'gloria_band',
    'n_tx': 'elwb_n_tx',
    'num_hops': 'elwb_num_hops',
}

################################################################################
# Helper functions
################################################################################

def getDfHash(df):
    '''Calculates a hash over all dataframe data values and the column labels, but not the index (aka row labels).
    '''
    colsBytes = ''.join(df.columns).encode('utf-8')
    colsArray = np.array((int(hashlib.sha256(colsBytes).hexdigest()[:16], 16))).astype(np.uint64)
    dfValuesArray = pd.util.hash_pandas_object(df, index=False).values
    l = np.append(colsArray, dfValuesArray)
    return hashlib.sha256(l).hexdigest()

def getJson(text, obsId=None):
    '''Find an convert json in a single line from serial output. Returns None if no valid json could be found.
    '''
    retJson = None
    retJsonParsingFailed = False
    # find index
    idx = None
    for i in range(len(text)):
        if text[i] == '{':
            idx = i
            break

    if not idx is None:
        try:
            retJson =  json.loads(text[idx:], strict=False)
        except json.JSONDecodeError:
            nodeText = ' ( observer {})'.format(obsId) if (not obsId is None) else ''
            retJsonParsingFailed = True
            print('WARNING: json could not be parsed{}: {}'.format(nodeText, text[idx:]))

    return retJson, retJsonParsingFailed


def readTypedefEnum(typeName, filePath, replaceName=None):
    cwd = os.path.dirname(os.path.realpath(__file__))
    with open(os.path.join(cwd, filePath), 'r') as f:
        text = f.read()

    ret = re.findall(r'^\s*typedef\s+enum\s*{{([^}}]+)}}\s*{typeName};'.format(typeName=typeName), text, re.MULTILINE)
    if len(ret) == 0:
        raise Exception('ERROR: typeName "{}" not found'.format(symbol))
    if len(ret) >= 2:
        raise Exception('ERROR: typeName "{}" defined multiple times!'.format(symbol))
    ret = ret[0].strip()
    # remove comments
    ret = re.sub(
           r'(\s*//[^\n\r]*)',
           '',
           ret
    )
    # remove line breaks
    ret = ret.replace('\n', '').replace('\r', '')
    # convert to list
    ret = [e.strip() for e in ret.split(',') if len(e)]
    # split elements into name and idx
    ret = [(e.split('=')[0], int(e.split('=')[1]) if len(e.split('=')) > 1 else None) for e in ret]
    retFinal = {}
    for e in ret:
        name, idx = e
        if not replaceName is None:
            name = name.replace(*replaceName)
        if not idx is None:
            retFinal[idx] = name
        else:
            if len(retFinal) == 0:
                retFinal[0] = name
            else:
                retFinal[list(retFinal.keys())[-1]+1] = name
    return retFinal

################################################################################
# Extract data functions
################################################################################

def extractData(testId, testDir=os.getcwd()):
    # directly read file from archive (without extracting to a file)
    archivePath = os.path.join(testDir, 'flocklab_testresults_{}.tar.gz'.format(testId))
    with tarfile.open(archivePath) as tar:
        # find correct file in archive
        serialFile = None
        testconfigFile = None
        for fn in tar.getnames():
            if 'testconfig.xml' in fn:
                testconfigFile = fn
            elif 'serial.csv' in fn:
                serialFile = fn

        df, imageConfigFromSerial = extractSerialData(tar.extractfile(tar.getmember(serialFile)))
        imageConfig = extractImageConfig(tar.extractfile(tar.getmember(testconfigFile)), imageConfigFromSerial)

    # add image config to df
    df['test_id'] = testId
    for k, v in imageConfig.items():
        df[k] = v

    # invalidate snr values for FSK modulations
    if imageConfig['modulation'] in FSK_MODULATIONS:
        df['snr'] = pd.NA
    df = df.astype({'snr': 'Int64'}) # make sure that data type of snr column stays int even after setting missing values to nan

    return df

def extractSerialData(serialFile):
    df = fl.serial2Df(serialFile, error='ignore')
    df.sort_values(by=['timestamp', 'observer_id'], inplace=True, ignore_index=True)

    # convert output with valid json to dict and remove other rows
    keepList = []
    failedList = []
    resList = []
    for idx, row in df.iterrows():
        jsonDict, jsonFailed = getJson(row['output'], row['observer_id'])
        keepList.append(1 if jsonDict else 0)
        failedList.append(1 if jsonFailed else 0)
        if jsonDict:
            resList.append(jsonDict)
    keepMask = np.asarray(keepList).astype(bool)
    dfd = df[keepMask].copy()
    dfd['data'] = resList
    failedMask = np.asarray(failedList).astype(bool)
    dfdFailed = df[failedMask]
    timeSpan = dfdFailed.timestamp.max() - dfdFailed.timestamp.min()
    if len(dfdFailed):
        print('A total of {} out of {} json-like rows spanning {:0.1f}s could not be parsed as JSON!'.format(len(dfdFailed), len(dfd)+len(dfdFailed), timeSpan))
    # # DEBUG
    # import matplotlib.pyplot as plt
    # plt.close('all')
    # fig, ax = plt.subplots()
    # tRef = df.timestamp.min()
    # ax.hist(dfdFailed.timestamp.to_numpy() - tRef, bins=50)
    # ax.axvline(df.timestamp.min() - tRef, c='k')
    # ax.axvline(df.timestamp.max() - tRef, c='k')
    # ax.set_xlabel('Time')
    # ax.set_ylabel('Count')

    # figure out list of nodes available in the serial trace
    nodeList = list(set(dfd.observer_id))
    numNodes = len(nodeList)

    print('nodeList: {}'.format(nodeList))
    print('numNodes: {}'.format(numNodes))

    # split dfd based on the two types of json objects (imageConfig vs floodRx)
    imageConfigDf = dfd[['node_id' in e.keys() for e in dfd.data]]
    floodRxDf = dfd[['rx_cnt' in e.keys() for e in dfd.data]]
    # sanity checks
    diffDf = dfd[np.logical_and([not 'node_id' in e.keys() for e in dfd.data], [not 'rx_cnt' in e.keys() for e in dfd.data])]
    assert len(imageConfigDf) + len(floodRxDf) == len(dfd)
    for idx, row in imageConfigDf.iterrows():
        assert row['observer_id'] == row['data']['node_id']

    # add values from data dict to main table (i.e. create flat table)
    floodRxDfList = floodRxDf.to_dict('records')
    floodRxDfListNew = []
    for floodRxDfDict in floodRxDfList:
        dataDict = floodRxDfDict['data']

        # invalidate data based on rx_cnt and t_ref_updated
        invalidateList = []
        if dataDict['rx_cnt'] == 0:
            invalidateList = ['rx_idx', 'snr', 'rssi', 'payload_len', 't_ref_updated', 'network_time', 't_ref']
        elif 't_ref_updated' in dataDict and dataDict['t_ref_updated'] == 0:
            invalidateList = ['network_time', 't_ref']
        for k in invalidateList:
            dataDict[k] = None

        for k,v in dataDict.items():
            floodRxDfDict[k] = v
        floodRxDfListNew.append(floodRxDfDict)
    # use Int for int data even if column contains nan -> use pandas 'Int64'
    floodRxDfNew = pd.DataFrame.from_dict(floodRxDfListNew)
    # FIXME: column data type should be passed to constructor not applied later with astype() but this does not seem to be supported yet...
    intCols = ['rx_idx', 'snr', 'rssi', 'payload_len', 't_ref_updated', 'network_time', 't_ref']
    floodRxDfNew = floodRxDfNew.astype(dict((col, 'Int64') for col in intCols))

    return floodRxDfNew, imageConfigDf.data.tolist()


def extractImageConfig(testconfigFile, imageConfigFromSerial=[]):
    imageConfig = {}
    if not imageConfigFromSerial:
        ## old way to pass imageConfig (via custom field of xml test config); for backwards compatibility
        print('Reading imageConfig from custom field of testconfig xml...')
        customText = fl.getCustomField(testconfigFile)
        try:
            custom = json.loads(customText)
        except Exception as e:
            raise Exception('ERROR: Unable to read custom field: {}'.format(e))

        configMacros = custom['imageConfig'] if 'imageConfig' in custom else None
        imagePatchingDict = custom['imagePatchingDict'] if 'imagePatchingDict' in custom else None

        for imageConfigName in imageConfigToMacro.keys():
            if imagePatchingDict and (imageConfigToGlobalVar[imageConfigName] in imagePatchingDict):
                imageConfig[imageConfigName] = imagePatchingDict[imageConfigToGlobalVar[imageConfigName]]
            else:
                imageConfig[imageConfigName] = configMacros[imageConfigToMacro[imageConfigName]]
    else:
        ## new way to pass image Config (via json output in serial)
        print('Reading imageConfig from serial...')
        # copy values from first list element (and ignore not needed values)
        for imageConfigName in imageConfigToGlobalVar.keys():
            imageConfig[imageConfigName] = imageConfigFromSerial[0][imageConfigName]
        # sanity check: ensure first list element is equal to all other list elements
        for d in imageConfigFromSerial:
            for k in imageConfigToGlobalVar.keys():
                assert d[k] == imageConfig[k]

    # Print debug info
    print('=== imageConfig ===')
    for imageConfigName, val in imageConfig.items():
        print('{}={}'.format(imageConfigName, val))

    return imageConfig

################################################################################
# Main
################################################################################

if __name__ == "__main__":
    # # check arguments
    # if len(sys.argv) < 2:
    #     print("no test number specified!")
    #     sys.exit(1)
    # # obtain list of tests
    # testIdList = map(int, sys.argv[1:])


    # testDir = '/home/rtrueb/polybox/PhD/Projects/FlockLab2/flocklab_tests'
    # testIdList = [3048, 3077]
    # testIdList = [3081, 3082]
    # testIdList = [3559]
    # testIdList = [3560, 3561, 3562] # dataset debug
    
    # sample dataset
    # testIdList = [3128, 3129, 3130] 

    # # dataset1
    # testDir = './data/dataset1/'
    # testIdList = range(3582, 3601+1)

    # dataset2
    testDir = './data/dataset2/'
    testIdList =  []
    testIdList += list(range(3823, 3842+1))
    for tId in [3826, 3837]: testIdList.remove(tId) # many rows with corrupte chars in serial log
    testIdList += list(range(3912, 3943+1))
    testIdList.remove(3918) # 4 non-parsable json rows
    testIdList.remove(3926) # test failed to run
    testIdList += list(range(3947, 3958+1))
    testIdList.remove(3952) # 1 non-parsable json row
    testIdList += [3965] # rerun of 3952

    # # debug dataset2
    # testIdList = [3826] # tx_power=-9, modulation=8, serial corrupted
    # testIdList = [3837] # tx_power=9, modulation=10, serial corrupted
    # testIdList = range(3860, 3865+1)
    # testIdList = range(3880, 3883+1)

    # ensure dir for storing tests exists
    os.makedirs(testDir, exist_ok=True)

    # obtain map to map elwb_phase enum idx to name
    elwbPhases = readTypedefEnum('elwb_phases_t', '../Lib/protocol/elwb/elwb.h', replaceName=('ELWB_PHASE_', ''))

    # extract and store data for all tests
    dfList = []
    for testId in testIdList:
        print('===== testId={} ====='.format(testId))

        # download test results if not available already
        if not os.path.isfile(os.path.join(testDir, 'flocklab_testresults_{}.tar.gz'.format(testId))):
            print('Downloading FL2 test {}'.format(testId))
            fl.getResults(testId, testDir, extract=False)

        # extract data from serial.csv and testconfig.xml
        df = extractData(testId, testDir=testDir)

        # map elwb_phase enum idx to names
        df['elwb_phase_mapped'] = df.elwb_phase.map(lambda x: elwbPhases[x])
        df.drop('elwb_phase', axis='columns', inplace=True)
        df.rename(columns={'elwb_phase_mapped': 'elwb_phase'}, inplace=True)

        # reduce to 1 node ID (use observer ID sice node ID is not actively used but call it node ID)
        df.drop('node_id', axis='columns', inplace=True)
        df.rename(columns={'observer_id': 'node_id'}, inplace=True)

        dfList.append(df)

    dfAll = pd.concat(dfList)

    # select columns to keep (with correct ordering)
    columns = [
        'test_id',        # src: testbed
        'timestamp',
        'node_id',
        'host_id',        # src: settings
        'tx_power',
        'modulation',
        'rf_band',
        'n_tx',
        'num_hops',
        'initiator_id',   # src: node
        'elwb_phase',
        'rx_cnt',
        'rx_idx',
        'rx_started',
        'rssi',
        'snr',
        'payload_len',
        'network_time',
        't_ref',
    ]
    dfDataset = dfAll[columns]

    dfHash = getDfHash(dfDataset)

    print('===== hexHash =====')
    print(dfHash)

    outputDir = './data/' if 'flocklab_tests' in testDir else os.path.split(testDir)[0]
    fileName = 'flood_dataset_{maxTestId}_{hash}.{{ext}}'.format(maxTestId=max(testIdList), hash=dfHash[:8])

    # save data to file
    # dfDataset.to_csv(
    #     path_or_buf=path=os.path.join(outputDir, fileName.format(ext='csv'),
    #     index=False,
    #     header=True,
    # )
    dfDataset.to_pickle(
        path=os.path.join(outputDir, fileName.format(ext='zip')),
    )

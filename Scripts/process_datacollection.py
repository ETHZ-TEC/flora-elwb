#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 20210216

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
    'num_hops': 'ELWB_NUM_HOPS',
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

def getJson(text):
    '''Find an convert json in a single line from serial output. Returns None if no valid json could be found.
    '''
    ret = None
    # find index
    idx = 0
    if not '{' in text:
        return ret
    for i in range(len(text)):
        if text[i] == '{':
            idx = i
            break

    try:
        ret =  json.loads(text[idx:], strict=False)
    except json.JSONDecodeError:
        print('WARNING: json could not be parsed: {}'.format(text[idx:]))
    return ret


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
    df, imageConfigFromSerial = extractSerialData(testId, testDir)
    imageConfig = extractImageConfig(testId, testDir, imageConfigFromSerial)

    df['test_id'] = testId
    for k, v in imageConfig.items():
        df[k] = v

    # invalidate snr values for FSK modulations
    if imageConfig['modulation'] in FSK_MODULATIONS:
        df['snr'] = pd.NA
    df = df.astype({'snr': 'Int64'}) # make sure that data type of snr column stays int even after setting missing values to nan

    return df

def extractSerialData(testId, testDir=os.getcwd()):
    serialPath = os.path.join(testDir, "{}/serial.csv".format(testId))

    # # download test results if directory does not exist
    # if not os.path.isfile(serialPath):
    #     fl.getResults(testId)

    df = fl.serial2Df(serialPath, error='ignore')
    df.sort_values(by=['timestamp', 'observer_id'], inplace=True, ignore_index=True)

    # convert output with valid json to dict and remove other rows
    keepMask = []
    resList = []
    for idx, row in df.iterrows():
        jsonDict = getJson(row['output'])
        keepMask.append(1 if jsonDict else 0)
        if jsonDict:
            resList.append(jsonDict)
    dfd = df[np.asarray(keepMask).astype(bool)].copy()
    dfd['data'] = resList

    # figure out list of nodes available in the serial trace
    nodeList = list(set(dfd.observer_id))
    numNodes = len(nodeList)

    print('nodeList: {}'.format(nodeList))
    print('numNodes: {}'.format(numNodes))

    # split dfd based on the two types of json objects (imageConfig vs floodRx)
    imageConfigDf = dfd[['node_id' in e.keys() for e in dfd.data]]
    floodRxDf = dfd[['rx_cnt' in e.keys() for e in dfd.data]]
    # sanity checks
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


def extractImageConfig(testId, testDir=os.getcwd(), imageConfigFromSerial=[]):
    imageConfig = {}
    if not imageConfigFromSerial:
        print('Reading imageConfig from custom field of testconfig xml...')
        ## old way to pass imageConfig (via custom field of xml test config); for backwards compatibility
        customText = fl.get_custom_field(os.path.join(testDir, str(testId)))
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
    TESTDIR = '/home/rtrueb/polybox/PhD/Projects/FlockLab2/flocklab_tests'
    # TESTDIR = './data/dataset1/'
    
    # # check arguments
    # if len(sys.argv) < 2:
    #     print("no test number specified!")
    #     sys.exit(1)
    # # obtain list of tests
    # testIdList = map(int, sys.argv[1:])

    # testIdList = [3048, 3077]
    # testIdList = [3081, 3082]
    # testIdList = [3128, 3129, 3130] # sample dataset
    # testIdList = [3559]
    # testIdList = [3560, 3561, 3562] # dataset debug
    # testIdList = range(3582, 3601+1) # dataset1
    testIdList = [3651] # debug

    # obtain map to map elwb_phase enum idx to name
    elwbPhases = readTypedefEnum('elwb_phases_t', '../Lib/protocol/elwb/elwb.h', replaceName=('ELWB_PHASE_', ''))

    # extract and store data for all tests
    dfList = []
    for testId in testIdList:
        print('===== testId={} ====='.format(testId))

        # download test results if not available already
        if not os.path.isdir(os.path.join(TESTDIR, str(testId))):
            print('Downloading FL2 test {}'.format(testId))
            fl.getResults(testId, TESTDIR)

        # extract data from serial.csv and testconfig.xml
        df = extractData(testId, testDir=TESTDIR)

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

    # save data to file
    dfDataset.to_csv(
        path_or_buf='./data/flood_dataset_{}_{}.csv'.format(max(testIdList), dfHash[:8]),
        index=False,
        header=True,
    )
    dfDataset.to_pickle(
        path='./data/flood_dataset_{}_{}.zip'.format(max(testIdList), dfHash[:8]),
    )

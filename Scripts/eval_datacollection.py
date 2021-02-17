#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 20210216

@author: romantrueb
@brief:  Evaluate data collected by flooding using eLWB
"""

import sys
import os
import numpy as np
import pandas as pd
import json
from collections import OrderedDict
import pickle

from flocklab import Flocklab
from flocklab import *

fl = Flocklab()



################################################################################
TESTDIR = '/home/rtrueb/polybox/PhD/Projects/FlockLab2/flocklab_tests'


################################################################################
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

################################################################################

def extractData(testNo, testDir=os.getcwd()):
    print('testNo: {}'.format(testNo))
    serialPath = os.path.join(testDir, "{}/serial.csv".format(testNo))


    # # download test results if directory does not exist
    # if not os.path.isfile(serialPath):
    #     fl.getResults(testNo)

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

    # add values from data dict to main table (i.e. create flat table)
    dfdList = dfd.to_dict('records')
    dfdListNew = []
    for dfdDict in dfdList:
        dataDict = dfdDict['data']
        for k,v in dataDict.items():
            k = 'network_time' if k == 'timestamp' else k # workaround for first format (with overlapping column names)
            dfdDict[k] = v
        dfdListNew.append(dfdDict)
    dfdNew = pd.DataFrame(dfdListNew)

    return dfdNew

    # # prepare
    # groups = dfd.groupby('observer_id')
    # prrMatrix = np.empty( (numNodes, numNodes,) ) * np.nan       # packet reception ratio (PRR)
    # crcErrorMatrix = np.empty( (numNodes, numNodes,) ) * np.nan  # ratio of packets with CRC error
    # pathlossMatrix = np.empty( (numNodes, numNodes,) ) * np.nan  # path loss
    #
    # # Get TestConfig and RadioConfig & check for consistency
    # testConfigDict = OrderedDict()
    # radioConfigDict = OrderedDict()
    # for node in nodeList:
    #     testConfigFound = False
    #     radioConfigFound = False
    #     testConfigDict[node] = None
    #     radioConfigDict[node] = None
    #     gDf = groups.get_group(node)
    #     for d in gDf.data.to_list():
    #         if d['type'] == 'TestConfig':
    #             testConfigDict[node] = d
    #             testConfigFound = True
    #         if d['type'] == 'RadioConfig':
    #             radioConfigDict[node] = d
    #             radioConfigFound = True
    #         if testConfigFound and radioConfigFound:
    #             break
    #
    # for node in nodeList:
    #     assert testConfigDict[nodeList[0]] == testConfigDict[node]
    #     assert radioConfigDict[nodeList[0]] == radioConfigDict[node]
    #
    # testConfig = testConfigDict[nodeList[0]]
    # radioConfig = radioConfigDict[nodeList[0]]
    #
    # # Make sure that round boundaries do not overlap
    # if not assertionOverride:
    #     currentSlot = -1
    #     for d in dfd.data.to_list():
    #         if d['type'] == 'StartOfRound':
    #             node = d['node']
    #             # print('Start: {}'.format(node))
    #             assert node >= currentSlot
    #             if node > currentSlot:
    #                 currentSlot = node
    #         elif d['type'] == 'EndOfRound':
    #             node = d['node']
    #             # print('End: {}'.format(node))
    #             assert node >= currentSlot
    #
    # # extract statistics (PRR, path loss, ...)
    # # iterate over rounds
    # for roundIdx, roundNo in enumerate(nodeList):
    # # for roundNo in [nodeList[0]]:
    #     # print('Round: {}'.format(roundNo))
    #     txNode = roundNo
    #     txNodeIdx = roundIdx
    #     numTx = 0
    #     numRxDict = OrderedDict()
    #     numCrcErrorDict = OrderedDict()
    #     rssiAvgDict = OrderedDict()
    #     # iterate over nodes
    #     for nodeIdx, node in enumerate(nodeList):
    #         rows = getRows(roundNo, groups.get_group(node))
    #         if node == txNode:
    # #            print(node)
    #             txDoneList = [elem for elem in rows if (elem['type']=='TxDone')]
    #             numTx = len(txDoneList)
    # #            print(numTx, testConfig['numTx'])
    #             assert numTx == testConfig['numTx']
    #         else:
    #             rxDoneList = [elem for elem in rows if (elem['type']=='RxDone' and elem['key']==testConfig['key'] and elem['crc_error']==0)]
    #             crcErrorList = [elem for elem in rows if (elem['type']=='RxDone' and elem['crc_error']==1)]
    #             numRxDict[node] = len(rxDoneList)
    #             numCrcErrorDict[node] = len(crcErrorList)
    #             rssiAvgDict[node] = np.mean([elem['rssi'] for elem in rxDoneList]) if len(rxDoneList) else np.nan
    #
    # # save obtained data to file (including nodeList to resolve idx <-> node ID relations)
    # pklPath = './data/linktest_data_{}.pkl'.format(testNo)
    # os.makedirs(os.path.split(pklPath)[0], exist_ok=True)
    # with open(pklPath, 'wb' ) as f:
    #     d = {
    #         'testConfig': testConfig,
    #         'radioConfig': radioConfig,
    #         'nodeList': nodeList,
    #         'prrMatrix': prrMatrix,
    #         'crcErrorMatrix': crcErrorMatrix,
    #         'pathlossMatrix': pathlossMatrix,
    #     }
    #     pickle.dump(d, f)


if __name__ == "__main__":
    # # check arguments
    # if len(sys.argv) < 2:
    #     print("no test number specified!")
    #     sys.exit(1)
    # # obtain list of tests
    # testNoList = map(int, sys.argv[1:])

    # testNoList = [2917]
    # testNoList = [2921]
    testNoList = [2930]

    # extract data for all tests
    for testNo in testNoList:
        df = extractData(testNo, testDir=TESTDIR)
        df.to_csv(
            path_or_buf='./datacollection_{}.csv'.format(testNo),
            columns=[
                'timestamp',
                'observer_id',
                'node_id',
                'initiator',
                'elwb_phase',
                'rx_cnt',
                'rx_idx',
                'rx_started',
                'rssi',
                'snr',
                'network_time',
                't_ref',
            ],
            index=False,
            header=True,
        )

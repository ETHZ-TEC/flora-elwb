#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 20210225

@author: romantrueb
@brief:  Evaluate data collected by flooding using eLWB
"""

import sys
import os
import numpy as np
import pandas as pd
from collections import OrderedDict
import pickle
import hashlib
import re

# graphs
import networkx as nx
from bokeh import plotting
from bokeh.layouts import column
from bokeh.models import ColumnDataSource, LabelSet, Band

# construct html with python
import dominate
from dominate.tags import *
from dominate.util import raw


################################################################################
FSK_MODULATIONS = [8, 9, 10]

################################################################################
# Helper functions/objects
################################################################################

def getDfHash(df):
    '''Calculates a hash over all dataframe data values and the column labels, but not the index (aka row labels).
    '''
    colsBytes = ''.join(df.columns).encode('utf-8')
    colsArray = np.array((int(hashlib.sha256(colsBytes).hexdigest()[:16], 16))).astype(np.uint64)
    dfValuesArray = pd.util.hash_pandas_object(df, index=False).values
    l = np.append(colsArray, dfValuesArray)
    return hashlib.sha256(l).hexdigest()

def styleDf(df, cmap='inferno', format='{:.1f}', replaceNan=True, applymap=None):
    ret = ( df.style
            .background_gradient(cmap=cmap, axis=None)
            .format(format) )
    if applymap is not None:
        ret = ret.applymap(applymap)

    ret = ret.render()

    if replaceNan:
        ret = ret.replace('nan','')

    return ret

htmlStyleBlock = '''
    table, th, td {font-size:10pt; border:1px solid lightgrey; border-collapse:collapse; text-align:left; font-family:arial;}
    th, td {padding: 5px; text-align:center; width:22px;}
    table.outer, th.outer, td.outer {font-size:10pt; border:0px solid lightgrey; border-collapse:collapse; text-align:left; font-family:arial;}
    th.outer, td.outer {padding: 5px; text-align:center;}
'''

################################################################################
# Functions
################################################################################

def extractConnData(df, txConfigLabels):
    # get all node IDs
    nodeIds = sorted(df.node_id.unique())
    numNodes = len(nodeIds)

    # extract data
    matrixDfDict = OrderedDict()
    for txConfig, dfMod in df.groupby(by=txConfigLabels):
        configStr = ', '.join(['{}={}'.format(k, v) for k, v in zip(txConfigLabels, txConfig)])
        print('=== {} ==='.format(configStr))

        # prepare matrices
        frrMatrix            = np.empty( (numNodes, numNodes,) ) * np.nan    # flood reception ratio (FRR) for each connection
        prrMatrix            = np.empty( (numNodes, numNodes,) ) * np.nan    # packet reception ratio (PRR) for each connection
        hopDistanceMatrix    = np.empty( (numNodes, numNodes,) ) * np.nan    # hop distance for each connection
        numFloodsMatrix      = np.empty( (numNodes, numNodes,) ) * np.nan    # number of floods per connection
        numFloodsSuccMatrix  = np.empty( (numNodes, numNodes,) ) * np.nan    # number successfully received floods per connection

        dfModTmp = dfMod[(dfMod.elwb_phase!='CONT') & (dfMod.elwb_phase!='REQ')]   # ignore floods during contention phase since content is unreliable in parts
        for conn, dfConn in dfModTmp.groupby(by=['initiator_id', 'node_id']):
            nodeTx, nodeRx = conn

            # skip flood log entries where nodes have not received the nodeId of the host
            if nodeTx==0 or nodeRx==0:
                continue

            nodeTxIdx = nodeIds.index(nodeTx)
            nodeRxIdx = nodeIds.index(nodeRx)
            numFloods = len(dfConn)
            numFloodsSucc = np.sum(dfConn.rx_cnt > 0)
            # numFailed = np.sum(dfConn.rx_cnt == 0)
            numFloodsMatrix[nodeTxIdx, nodeRxIdx] = numFloods
            numFloodsSuccMatrix[nodeTxIdx, nodeRxIdx] = numFloodsSucc
            frrMatrix[nodeTxIdx, nodeRxIdx] = numFloodsSucc/numFloods
            hopDistance = np.mean(dfConn[dfConn.notna().rx_idx].rx_idx)
            if not pd.isnull(hopDistance): # note: it is not possible to assign pd.NA to a numpy matrix
                hopDistanceMatrix[nodeTxIdx][nodeRxIdx] = hopDistance
            assert len(dfConn[dfConn.notna().rx_idx]) == numFloodsSucc # this needs to hold, otherwise we cannot use numFloodsSucc to determine number of floods used to calculate avg hopDistance

        dfModTmp = dfMod[(dfMod.elwb_phase!='CONT') & (dfMod.elwb_phase!='REQ') & (dfMod.rx_idx==0)]   # ignore floods during contention phase & and filter out floods with rx_idx > 0
        for conn, dfConn in dfModTmp.groupby(by=['initiator_id', 'node_id']):
            nodeTx, nodeRx = conn

            # skip flood log entries where nodes have not received the nodeId of the host
            if nodeTx==0 or nodeRx==0:
                continue

            nodeTxIdx = nodeIds.index(nodeTx)
            nodeRxIdx = nodeIds.index(nodeRx)

            numFloodsFirstSlot = len(dfConn)
            prrMatrix[nodeTxIdx, nodeRxIdx] = numFloodsFirstSlot/numFloodsMatrix[nodeTxIdx, nodeRxIdx]

        # convert numpy matrix to pandas dataframe to add correct indices
        matrixDfDict[txConfig] = {
            'prr':           pd.DataFrame(data=prrMatrix, index=nodeIds, columns=nodeIds),
            'frr':           pd.DataFrame(data=frrMatrix, index=nodeIds, columns=nodeIds),
            'hopDistance':   pd.DataFrame(data=hopDistanceMatrix, index=nodeIds, columns=nodeIds),
            'numFloods':     pd.DataFrame(data=numFloodsMatrix, index=nodeIds, columns=nodeIds),
            'numFloodsSucc': pd.DataFrame(data=numFloodsSuccMatrix, index=nodeIds, columns=nodeIds),
        }

    return matrixDfDict

def saveFrrHtml(matrixDfDict, txConfigLabels, dfHash):
    h = html()
    with h.add(head()):
        meta(charset='UTF-8')
        style(raw(htmlStyleBlock))
    with h.add(body()).add(div(id='content')):
        h1('Flood Reception Ratio (FRR)')
        for txConfig in matrixDfDict.keys():
            frr_html = styleDf(
                df=matrixDfDict[txConfig]['frr'],
                cmap='inferno',
                format='{:.1f}',
            )
            numFloods_html = styleDf(
                df=matrixDfDict[txConfig]['numFloods'],
                cmap='YlGnBu',
                format='{:.0f}',
                applymap=lambda x: 'background: white' if pd.isnull(x) else '',
            )
            configStr = ', '.join(['{}={}'.format(k, v) for k, v in zip(txConfigLabels, txConfig)])
            h2(configStr)
            with table(cls="outer").add(tbody()):
                with tr(cls="outer"):
                    th('FRR Matrix', cls='outer')
                    th(cls="outer")
                    th('Number of Floods', cls='outer')
                with tr(cls='outer'):
                    td(raw(frr_html), cls='outer')
                    td(cls="outer")
                    td(raw(numFloods_html), cls='outer')

    htmlPath = './data/frr_{}.html'.format(dfHash[:8])
    os.makedirs(os.path.split(htmlPath)[0], exist_ok=True)
    with open(htmlPath,"w") as fp:
       fp.write(h.render())

def saveHopDistanceHtml(matrixDfDict, txConfigLabels, dfHash):
    h = html()
    with h.add(head()):
        meta(charset='UTF-8')
        style(raw(htmlStyleBlock))
    with h.add(body()).add(div(id='content')):
        h1('Hop Distance')
        for txConfig in matrixDfDict.keys():
            frr_html = styleDf(
                df=matrixDfDict[txConfig]['hopDistance'],
                cmap='inferno_r',
                format='{:.1f}',
                applymap=lambda x: 'background: white' if pd.isnull(x) else '',
            )
            numFloods_html = styleDf(
                df=matrixDfDict[txConfig]['numFloodsSucc'],
                cmap='YlGnBu',
                format='{:.0f}',
                applymap=lambda x: 'background: white' if pd.isnull(x) else '',
            )
            configStr = ', '.join(['{}={}'.format(k, v) for k, v in zip(txConfigLabels, txConfig)])
            h2(configStr)
            with table(cls="outer").add(tbody()):
                with tr(cls="outer"):
                    th('Hop Distance Matrix (avg rx_idx)', cls='outer')
                    th(cls="outer")
                    th('Number of Successfully Received Floods', cls='outer')
                with tr(cls='outer'):
                    td(raw(frr_html), cls='outer')
                    td(cls="outer")
                    td(raw(numFloods_html), cls='outer')

    htmlPath = './data/hopDistance_{}.html'.format(dfHash[:8])
    os.makedirs(os.path.split(htmlPath)[0], exist_ok=True)
    with open(htmlPath,"w") as fp:
       fp.write(h.render())

def savePrrHtml(matrixDfDict, txConfigLabels, dfHash):
    h = html()
    with h.add(head()):
        meta(charset='UTF-8')
        style(raw(htmlStyleBlock))
    with h.add(body()).add(div(id='content')):
        h1('Packet Reception Ratio (PRR)')
        for txConfig in matrixDfDict.keys():
            frr_html = styleDf(
                df=matrixDfDict[txConfig]['prr'],
                cmap='inferno',
                format='{:.1f}',
            )
            numFloods_html = styleDf(
                df=matrixDfDict[txConfig]['numFloods'],
                cmap='YlGnBu',
                format='{:.0f}',
                applymap=lambda x: 'background: white' if pd.isnull(x) else '',
            )
            configStr = ', '.join(['{}={}'.format(k, v) for k, v in zip(txConfigLabels, txConfig)])
            h2(configStr)
            with table(cls="outer").add(tbody()):
                with tr(cls="outer"):
                    th('PRR Matrix (num of received floods with rx_idx=0)', cls='outer')
                    th(cls="outer")
                    th('Number of Floods', cls='outer')
                with tr(cls='outer'):
                    td(raw(frr_html), cls='outer')
                    td(cls="outer")
                    td(raw(numFloods_html), cls='outer')

    htmlPath = './data/prr_{}.html'.format(dfHash[:8])
    os.makedirs(os.path.split(htmlPath)[0], exist_ok=True)
    with open(htmlPath,"w") as fp:
       fp.write(h.render())

def evalConnectivity(matrixDfDict, nodeIds, txConfigLabels, prrThreshold=0.95):
    print('==== getConnectivity ====')
    plotDict = {}
    nodeDegreeDict = {}
    for txConfig, d in matrixDfDict.items():
        prrMatrix = d['prr']
        configStr = ', '.join(['{}={}'.format(k, v) for k, v in zip(txConfigLabels, txConfig)])
        print('=== {} ==='.format(configStr))
        # consruct graph from PRR data
        g = nx.Graph()
        for node1 in nodeIds:
            g.add_node(node1) # ensure that all nodes are contained in the graph
            for node2 in nodeIds:
                if node1 == node2:
                    continue
                # connection is only considered if PRR in both directions is higher than threshold
                prr1 = prrMatrix.loc[(node1, node2)]
                prr2 = prrMatrix.loc[(node2, node1)]
                if np.isnan(prr1) or np.isnan(prr2):
                    continue
                prrMin = min(prr1, prr2)
                if prrMin >= prrThreshold:
                    # Notes: Adding an edge that already exists updates the edge data
                    g.add_edge(node1, node2)

        ## plot the obtained graph
        configStr = ', '.join(['{}={}'.format(k, v) for k, v in zip(txConfigLabels, txConfig)])
        plot = plotting.figure(title='txConfig: {}'.format(configStr), x_range=(-1.1,1.1), y_range=(-1.1,1.1), plot_height=700, aspect_ratio=1)
        pos = nx.circular_layout(g)
        # pos = nx.fruchterman_reingold_layout(g)
        graph = plotting.from_networkx(g, pos, scale=2, center=(0,0))
        plot.renderers.append(graph)
        # add node labels to the graph
        x, y = zip(*graph.layout_provider.graph_layout.values())
        node_labels = list(g.nodes())
        source = ColumnDataSource({'x': x, 'y': y,  'label': node_labels})
        labels = LabelSet(x='x', y='y', text='label', source=source, background_fill_color='white')
        plot.renderers.append(labels)
        # collect
        plotDict[txConfig] = plot

        # determine connectivity metrics
        # networkEdegeConn = nx.edge_connectivity(g) # overall edge connectivity
        nodeDegree = {}
        for nodeId in nodeIds:
            nodeDegree[nodeId] = g.degree[nodeId]
        nodeDegreeDict[txConfig] = nodeDegree
        vals = list(nodeDegree.values())
        print('nodeDegree: min={:.2f}, mean={:.2f}, max={:.2f}'.format(np.min(vals), np.mean(vals), np.max(vals)))

    # save all network graphs to html
    htmlPath = './data/prr_connectivity_{}.html'.format(dfHash[:8])
    os.makedirs(os.path.split(htmlPath)[0], exist_ok=True)
    plotting.output_file(htmlPath)
    # plotting.save(column(list(plotDict.values())))
    plotting.show(column(list(plotDict.values()))) # DEBUG

    # plot nodeDegree data
    source = ColumnDataSource({
        'x': [e[txConfigLabels.index('tx_power')] for e in list(nodeDegreeDict.keys())],
        'y': [np.mean(list(e.values())) for e in nodeDegreeDict.values()],
        'upper': [np.min(list(e.values())) for e in nodeDegreeDict.values()],
        'lower': [np.max(list(e.values())) for e in nodeDegreeDict.values()],
    })
    p = plotting.figure()
    p.scatter(x='x', y='y', line_color='k', fill_alpha=0.3, size=5, source=source)
    band = Band(base='x', lower='lower', upper='upper', source=source, level='underlay',
                fill_alpha=1.0, line_width=1, line_color='black')
    p.add_layout(band)

    p.title.text = "Node Degree"
    p.xgrid[0].grid_line_color=None
    p.ygrid[0].grid_line_alpha=0.5
    p.xaxis.axis_label = 'Node Degree (min/avg/max)'
    p.yaxis.axis_label = 'tx_power config [dBm]'

    htmlPath = './data/connectivity_nodeDegree_{}.html'.format(dfHash[:8])
    os.makedirs(os.path.split(htmlPath)[0], exist_ok=True)
    plotting.output_file(htmlPath)
    # plotting.save(p)
    plotting.show(p) # DEBUG


################################################################################
# Main
################################################################################

if __name__ == "__main__":
    # # check arguments
    # if len(sys.argv) < 2:
    #     print("no dataset file specified!")
    #     sys.exit(1)
    # elif len(sys.argv) > 2:
    #     print("too many arguments provided!")
    #     sys.exit(1)
    # datasetFile = sys.argv[1]

    # datasetFile = '/home/rtrueb/gitlab/dpp/software/communication_platforms/sx126x_lora/flora/elwb/Scripts/data/flood_dataset_3562_3f8d4e67.zip'
    datasetFile = '/home/rtrueb/gitlab/dpp/software/communication_platforms/sx126x_lora/flora/elwb/Scripts/data/flood_dataset_3601_7c8def8b.zip'

    # load data from dataset
    df = pd.read_pickle(datasetFile)
    dfHash = getDfHash(df)

    # get all node IDs
    nodeIds = [int(e) for e in sorted(df.node_id.unique())] # explicitely convert to python int since bokeh from_networkx does not work with int64
    numNodes = len(nodeIds)

    ## extract and output matrix data

    # define txConfig labels which are used for grouping the results
    txConfigLabels = ['modulation', 'tx_power', 'n_tx', 'num_hops']

    # extract data per connection (Tx, Rx)
    print('==== extractConnData ====')
    matrixDfDict = extractConnData(df, txConfigLabels)

    ## save matrix data as colored matrices to html files
    saveFrrHtml(matrixDfDict, txConfigLabels, dfHash)
    saveHopDistanceHtml(matrixDfDict, txConfigLabels, dfHash)
    savePrrHtml(matrixDfDict, txConfigLabels, dfHash)

    ## extract and output graph data
    evalConnectivity(matrixDfDict, nodeIds, txConfigLabels)

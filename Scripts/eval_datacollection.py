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
@brief:  Evaluate data collected by flooding using eLWB
"""

import sys
import os
import numpy as np
import pandas as pd
from collections import OrderedDict, Counter
import pickle
import hashlib
import re

# plots
import networkx as nx
from bokeh import plotting
from bokeh.layouts import column, gridplot
from bokeh.models import ColumnDataSource, LabelSet, Div
from bokeh.palettes import Category10_10
import holoviews as hv
hv.extension('bokeh')
from holoviews import opts

# construct html with python
import dominate
from dominate.tags import *
from dominate.util import raw


################################################################################
FSK_MODULATIONS = [8, 9, 10]
PLOT_HEIGHT = 700
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

def extractConnectionData(df, txConfigLabels):
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

        # FRR and hop distance
        dfModTmp = dfMod[(dfMod.elwb_phase!='CONT') & (dfMod.elwb_phase!='REQ')]   # ignore floods during contention phase since content is unreliable in parts
        for conn, dfConn in dfModTmp.groupby(by=['initiator_id', 'node_id']):
            nodeTx, nodeRx = conn

            # skip flood log entries where nodes have not yet received the nodeId of the host
            if nodeTx==0 or nodeRx==0:
                continue

            # skip flood log entreis where which contain faulty data
            if not nodeTx in nodeIds:
                print('WARNING: out-of-range initiator_id {} observed!'.format(nodeTx))
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

        # PRR
        dfModTmp = dfMod[(dfMod.elwb_phase!='CONT') & (dfMod.elwb_phase!='REQ') & (dfMod.rx_idx==0)]   # ignore floods during contention phase & and filter out floods with rx_idx > 0
        for conn, dfConn in dfModTmp.groupby(by=['initiator_id', 'node_id']):
            nodeTx, nodeRx = conn

            # skip flood log entries where nodes have not received the nodeId of the host
            if nodeTx==0 or nodeRx==0:
                continue

            # skip flood log entreis where which contain faulty data
            if not nodeTx in nodeIds:
                print('WARNING: out-of-range initiator_id {} observed!'.format(nodeTx))
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


def saveMatricesToHtml(matrixDfDict, txConfigLabels, dfHash, matrixNames, titles, cmaps, formats, applymaps=None, outputDir='data'):
    h = html()
    if applymaps is None:
        applymaps = (lambda x: '', lambda x: '')
    with h.add(head()):
        meta(charset='UTF-8')
        style(raw(htmlStyleBlock))
    with h.add(body()).add(div(id='content')):
        # h1('Main Title')
        for txConfig in matrixDfDict.keys():
            html0 = styleDf(
                df=matrixDfDict[txConfig][matrixNames[0]],
                cmap=cmaps[0],
                format=formats[0],
                applymap=applymaps[0],
            )
            html1 = styleDf(
                df=matrixDfDict[txConfig][matrixNames[1]],
                cmap=cmaps[1],
                format=formats[1],
                applymap=applymaps[1],
            )
            configStr = ', '.join(['{}={}'.format(k, v) for k, v in zip(txConfigLabels, txConfig)])
            h2(configStr)
            with table(cls="outer").add(tbody()):
                with tr(cls="outer"):
                    th(titles[0], cls='outer')
                    th(cls="outer")
                    th(titles[1], cls='outer')
                with tr(cls='outer'):
                    td(raw(html0), cls='outer')
                    td(cls="outer")
                    td(raw(html1), cls='outer')

    htmlPath = os.path.join(outputDir, '{}_{}.html'.format(matrixNames[0], dfHash[:8]))
    os.makedirs(os.path.split(htmlPath)[0], exist_ok=True)
    with open(htmlPath,"w") as fp:
       fp.write(h.render())

def evalConnectivity(matrixDfDict, nodeIds, txConfigLabels, prrThreshold=0.95, outputDir='data'):
    print('==== evalConnectivity ====')
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
        plot = plotting.figure(title='txConfig: {}'.format(configStr), x_range=(-1.1,1.1), y_range=(-1.1,1.1), plot_height=PLOT_HEIGHT, aspect_ratio=1)
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
    htmlPath = os.path.join(outputDir, 'prr_connectivity_graph_{}.html'.format(dfHash[:8]))
    os.makedirs(os.path.split(htmlPath)[0], exist_ok=True)
    plotting.output_file(htmlPath)
    infoDiv = Div(text='prrThreshold={}'.format(prrThreshold))
    plotting.save(column([infoDiv] + list(plotDict.values())))

    ## plot nodeDegree data
    # create df with nodeDegree data
    nodeDegreeDf = pd.DataFrame()
    for idx, label in enumerate(txConfigLabels):
        nodeDegreeDf[label] = [e[idx] for e in list(nodeDegreeDict.keys())]
    nodeDegreeDf['nodeDegreeAvg'] = [np.mean(list(e.values())) for e in nodeDegreeDict.values()]
    nodeDegreeDf['nodeDegreeMax'] = [np.max(list(e.values())) for e in nodeDegreeDict.values()]
    nodeDegreeDf['nodeDegreeMin'] = [np.min(list(e.values())) for e in nodeDegreeDict.values()]
    nodeDegreeDf['nodeDegreeVals'] = [list(e.values()) for e in nodeDegreeDict.values()]
    # create all plots
    plotList = []
    aggP = plotting.figure(plot_height=PLOT_HEIGHT, plot_width=PLOT_HEIGHT) # aggregated plot
    color = Category10_10.__iter__()
    for modulation, groupDf in nodeDegreeDf.groupby(by=['modulation']):
        source = ColumnDataSource(groupDf)
        ## aggregated plot (avg only)
        col = next(color)
        aggP.line(x='tx_power', y='nodeDegreeAvg', source=source, legend_label='modulation={}'.format(modulation), line_color=col, )
        aggP.circle(x='tx_power', y='nodeDegreeAvg', source=source, legend_label='modulation={}'.format(modulation), color=col)
        ## violing plots
        # violin plot requires list keys and values (list of list is does not work out-of-the-box)
        kList = []
        vList = []
        for idx, row in groupDf.iterrows():
            kList += [row['tx_power']] * len(row['nodeDegreeVals'])
            vList += row['nodeDegreeVals']
        hp = hv.Violin((kList, vList), kdims='tx_power', vdims='nodeDegreeVals')
        hp.opts(title="Node Degree (modulation={})".format(modulation))
        hp.opts(opts.Violin(inner='stick', cut=0.1, bandwidth=0.1))
        p = hv.render(hp)
        p.plot_height=PLOT_HEIGHT
        p.plot_width=PLOT_HEIGHT
        p.xaxis.axis_label = 'tx_power config [dBm]'
        p.yaxis.axis_label = 'Node Degree'
        plotList.append(p)
    aggP.title.text = 'Node Degree (all modulations, avg only)'
    aggP.xgrid[0].grid_line_color=None
    aggP.ygrid[0].grid_line_alpha=0.5
    aggP.xaxis.axis_label = 'tx_power config [dBm]'
    aggP.yaxis.axis_label = 'Node Degree (avg)'
    aggP.legend.location = "top_left"
    aggP.legend.click_policy="hide"
    # plot all plots to a single HTML file
    htmlPath = os.path.join(outputDir, 'prr_connectivity_nodeDegree_{}.html'.format(dfHash[:8]))
    os.makedirs(outputDir, exist_ok=True)
    plotting.output_file(htmlPath)
    infoDiv = Div(text='prrThreshold={}'.format(prrThreshold))
    plotting.save(column([infoDiv, gridplot([aggP] + plotList, ncols=2)]))


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
    # datasetFile = '/home/rtrueb/gitlab/dpp/software/communication_platforms/sx126x_lora/flora/elwb/Scripts/data/flood_dataset_3601_7c8def8b.zip'
    # datasetFile = '/home/rtrueb/gitlab/dpp/software/communication_platforms/sx126x_lora/flora/elwb/Scripts/data/sample_dataset/flood_dataset_3130_43841b6f.zip'
    # datasetFile = '/home/rtrueb/gitlab/dpp/software/communication_platforms/sx126x_lora/flora/elwb/Scripts/data/dataset1/flood_dataset_3601_7814960e.zip'
    datasetFile = '/home/rtrueb/gitlab/dpp/software/communication_platforms/sx126x_lora/flora/elwb/Scripts/data/dataset2/flood_dataset_3965_0bbe0f03.zip'
    outputDir = os.path.split(datasetFile)[0]

    # load data from dataset
    df = pd.read_pickle(datasetFile)
    dfHash = getDfHash(df)

    # get all node IDs
    nodeIds = [int(e) for e in sorted(df.node_id.unique())] # explicitely convert to python int since bokeh from_networkx does not work with int64
    numNodes = len(nodeIds)

    # overview of available data
    print('==== Overview of available data ====')
    for modulation, grp in df.groupby(by=['modulation']):
        print('== modulation={} =='.format(modulation))
        c = Counter(grp.test_id.to_list())
        for k, v in dict(c).items():
            print('Test {}: {:>10} rows'.format(k, v))

    # define txConfig labels which are used for grouping the results
    txConfigLabels = ['modulation', 'tx_power', 'n_tx', 'num_hops']

    ## extract data per connection (Tx, Rx)
    print('==== extractConnectionData ====')
    matrixDfDict = extractConnectionData(df, txConfigLabels)

    ## save matrix data as colored matrices to html files
    # FRR
    saveMatricesToHtml(
        matrixDfDict,
        txConfigLabels,
        dfHash,
        matrixNames=('frr', 'numFloods'),
        titles=('FRR Matrix', 'Number of Floods'),
        cmaps=('inferno', 'YlGnBu'),
        formats=('{:.1f}', '{:.0f}'),
        applymaps=(lambda x: '', lambda x: 'background: white' if pd.isnull(x) else ''),
        outputDir=outputDir,
    )
    # hop distance
    saveMatricesToHtml(
        matrixDfDict,
        txConfigLabels,
        dfHash,
        matrixNames=('hopDistance', 'numFloodsSucc'),
        titles=('Hop Distance Matrix (avg rx_idx)', 'Number of Successfully Received Floods'),
        cmaps=('inferno_r', 'YlGnBu'),
        formats=('{:.1f}', '{:.0f}'),
        applymaps=(lambda x: 'background: white' if pd.isnull(x) else '', lambda x: 'background: white' if pd.isnull(x) else ''),
        outputDir=outputDir,
    )
    # PRR
    saveMatricesToHtml(
        matrixDfDict,
        txConfigLabels,
        dfHash,
        matrixNames=('prr', 'numFloods'),
        titles=('PRR Matrix (num of received floods with rx_idx=0)', 'Number of Floods'),
        cmaps=('inferno', 'YlGnBu'),
        formats=('{:.1f}', '{:.0f}'),
        applymaps=(lambda x: '', lambda x: 'background: white' if pd.isnull(x) else ''),
        outputDir=outputDir,
    )


    ## extract and output network graph and connectivity data
    evalConnectivity(matrixDfDict, nodeIds, txConfigLabels, prrThreshold=0.95, outputDir=outputDir)

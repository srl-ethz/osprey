# -*- coding: utf-8 -*-
# Copyright (C) OMG Plc 2017.
# All rights reserved.  This software is protected by copyright
# law and international treaties.  No part of this software / document
# may be reproduced or distributed in any form or by any means,
# whether transiently or incidentally to some other use of this software,
# without the written permission of the copyright owner.
"""
Visualise Retiming Results
"""

import argparse
import logging
import sys
import os
import csv
import copy

import dash
from dash.dependencies import Output, Event
import dash_core_components as dcc
import dash_html_components as html
import plotly
import plotly.graph_objs as go


colors = { 'background': '#CCCCEE', 'text': '#000000' }
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('RetimingViewer')



# list data sets
from sets import Set
subjects = Set()
from os import listdir
from os.path import isfile, join

with open( sys.argv[1], 'rb' ) as csvfile:
  csv_reader = csv.reader( csvfile, skipinitialspace=True )
  for row in csv_reader:
    if row[0] == 'Debug':
      if row[1] == 'Segment':
        subjects.add( row[2] + ":" + row[3])
      
subjects = sorted( subjects )

print "Parsing Subjects..."
for subject in subjects:
    print "subject", subject

# main dashboard layout
app = dash.Dash(__name__)
app.config['suppress_callback_exceptions']=True

app.layout = html.Div(id='main_div', style={'backgroundColor': colors['background']}, children=
        html.Div
        ([
            # heading
            html.H2(style={'textAlign': 'center', 'color': colors['text'], 'fontFamily': 'Sans-Serif'}, children = "Subjects:"),
            # Div for combo boxes
            html.Div
            ([
                # combo selector for run type
                html.Div( style={'width':'35%', 'display':'inline-block', 'margin-left':'10%', 'margin-bottom':'10'}, children=
                [
                    dcc.Dropdown(
                        id="data_set_selector",
                        options= [{'label': i, 'value': i} for i in subjects]
                        )       
               ])
            ]),
            # graphs
            dcc.Graph(id='Time'),
            dcc.Graph(id='TX'),
            dcc.Graph(id='TY'),
            dcc.Graph(id='TZ'),
            dcc.Graph(id='RX'),
            dcc.Graph(id='RY'),
            dcc.Graph(id='RZ'),
            dcc.Graph(id='RW')
        ])
        #, style={'width': '90%', 'fontFamily': 'Sans-Serif', 'margin-left': 'auto', 'margin-right': 'auto'}
    )


def build_segment_graph( selected_data_set, starting_index ):

    subject_segment = selected_data_set.split( ':' )
    
    sample_indices = []
    segment_values_p1 = []
    segment_values_p2 = []
    segment_values_p3 = []

    with open(sys.argv[1], 'rb') as csvfile:
        csv_reader = csv.reader(csvfile, skipinitialspace=True )
        for row in csv_reader:
            if row[0] == 'Debug':
              if row[1] == 'Segment':
                if row[2] == subject_segment[0] and row[3] == subject_segment[1]:
                  segment_values_p1.append( float( row[starting_index + 4] ) )
                  segment_values_p2.append( float( row[starting_index + 7] ) )
                  segment_values_p3.append( float( row[starting_index + 10] ) )

    
    num_samples = len( segment_values_p1 )

    traces = []
    
    traces.append( go.Scatter( x = range(0, num_samples), y = segment_values_p1, mode = 'lines',
                    name = 'First Sample', marker = dict( size='1', color='rgb(0, 0, 255)', opacity='0.3' ) ) )
    traces.append( go.Scatter( x = range(0, num_samples), y = segment_values_p2, mode = 'lines',
                    name = 'Second Sample', marker = dict( size='1', color='rgb(0, 255, 0)', opacity='0.3' ) ) )
    traces.append( go.Scatter( x = range(0, num_samples), y = segment_values_p3, mode = 'lines',
                    name = 'Predicted', marker = dict( size='1', color='rgb(255, 0, 0)', opacity='0.3' ) ) )

    return traces

#update graphs
@app.callback(
    dash.dependencies.Output('Time', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    subject_segment = selected_data_set.split( ':' )
    
    sample_indices = []
    tNow = []
    tp1 = []
    tp2 = []
    pT = []
    with open(sys.argv[1], 'rb') as csvfile:
        csv_reader = csv.reader(csvfile, skipinitialspace=True )
        for row in csv_reader:
            if row[0] == 'Debug':
              if row[1] == 'Predict':
                if row[2] == subject_segment[0]:
                  tNow.append( float( row[3] ) )
                  tp1.append( float( row[4] ) )
                  tp2.append( float( row[5] ) )
                  pT.append( float( row[6] ) )

    
    num_samples = len( tNow )

    traces = []
    
    traces.append( go.Scatter( x = range(0, num_samples), y = tNow, mode = 'lines',
                    name = 'Time Now', marker = dict( size='1', color='rgb(0, 0, 255)', opacity='0.3' ) ) )
    traces.append( go.Scatter( x = range(0, num_samples), y = tp1, mode = 'lines',
                    name = 'First Sample Time', marker = dict( size='1', color='rgb(0, 255, 0)', opacity='0.3' ) ) )
    traces.append( go.Scatter( x = range(0, num_samples), y = tp2, mode = 'lines',
                    name = 'Second Sample Time', marker = dict( size='1', color='rgb(255, 0, 0)', opacity='0.3' ) ) )
    traces.append( go.Scatter( x = range(0, num_samples), y = pT, mode = 'lines',
                    name = 'Prediction Time', marker = dict( size='1', color='rgb(0, 0, 0)', opacity='0.3' ) ) )

    fig_layout = go.Layout( height = 1000, title='Timing' )
    figure = dict( data=traces, layout=fig_layout )
    return figure

@app.callback(
    dash.dependencies.Output('TX', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    traces = build_segment_graph( selected_data_set, 0 )
    fig_layout = go.Layout( height = 1000, title='TX' )
    figure = dict( data=traces, layout=fig_layout )
    return figure

@app.callback(
    dash.dependencies.Output('TY', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    traces = build_segment_graph( selected_data_set, 1 )
    fig_layout = go.Layout( height = 1000, title='TY'  )
    figure = dict( data=traces, layout=fig_layout )
    return figure

@app.callback(
    dash.dependencies.Output('TZ', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    traces = build_segment_graph( selected_data_set, 2 )
    fig_layout = go.Layout( height = 1000, title='TZ'  )
    figure = dict( data=traces, layout=fig_layout )
    return figure

@app.callback(
    dash.dependencies.Output('RX', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    traces = build_segment_graph( selected_data_set, 3 )
    fig_layout = go.Layout( height = 1000, title='RX'  )
    figure = dict( data=traces, layout=fig_layout )
    return figure

@app.callback(
    dash.dependencies.Output('RY', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    traces = build_segment_graph( selected_data_set, 4 )
    fig_layout = go.Layout( height = 1000, title='RY'  )
    figure = dict( data=traces, layout=fig_layout )
    return figure

@app.callback(
    dash.dependencies.Output('RZ', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    traces = build_segment_graph( selected_data_set, 5 )
    fig_layout = go.Layout( height = 1000, title='RZ'  )
    figure = dict( data=traces, layout=fig_layout )
    return figure

@app.callback(
    dash.dependencies.Output('RW', 'figure'),
    [dash.dependencies.Input('data_set_selector', 'value')] )
def update_run_graph( selected_data_set ):
    traces = build_segment_graph( selected_data_set, 6 )
    fig_layout = go.Layout( height = 1000, title='RW'  )
    figure = dict( data=traces, layout=fig_layout )
    return figure




if __name__ == '__main__':
    app.run_server(debug=True)

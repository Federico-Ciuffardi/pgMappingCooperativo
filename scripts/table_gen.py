#!/usr/bin/env python3

import yaml
import sys
import os
import shutil
import json
import math
from collections import defaultdict
import time
from datetime import datetime

##########
# config #
##########
HOME=os.environ['HOME']

data_dir_root=f"{HOME}/catkin_ws/src/pgmappingcooperativo/data_wo_bags/"

default_mean_float_format="f"
default_mean_float_decimals=1

default_stddev_float_format= "f"
default_stddev_float_decimals = 1

def float2str(f,float_f,float_d):
    return "{:.{}{}}".format(f,float_d,float_f)


data_names = {
    "incrementalidad" : {-1 : "Construcción del GVD", 
                          0 : "Incremental", 
                          1 : "No incremental"},

    "ident_obj"       : {-1 : "Identificación de objetivos",
                          0 : "Simplificación de fronteras basada en cubrimiento", 
                          1 : "Simplificación de fronteras basada en K-Means",
                          2 : "Fronteras sin simplificar"},

    "desconocido"     : {-1 : "Consideración del espacio desconocido",
                          0 : "Desconocidas no propagan olas y $UF \\subseteq CGen$", 
                          1 : "Desconocidas se consideran libres"},

    "planificacion"   : {-1 : "Planificación",
                          0 : "Jerárquica", 
                          1 : "Sobre el GVD"},

    "todo"            : {-1 : "Variante",
                          0 : "Propuesta sin cambios",
                          1 : "No incremental",
                          2 : "Simplificación de fronteras basada en K-Means",
                          3 : "Fronteras sin simplificar",
                          4 : "Desconocidas se consideran libres"}

    }

x_labels = { "robot" : "Cantidad de robots", 
             "celda" : "$\\frac{celdas}{m^2}$"}

xs = { "robot" : [1,2,3,4,5],
       "celda" : [1.0,0.5,0.333333,0.25]}

translations = { "auction_info_time_diff_max"               : "Incremento máximo del tiempo en obtención de información $(s)$", 
                 "auction_info_time_diff_mean"              : "Incremento promedio del tiempo en obtención de información $(s)$",
                 "auction_info_time_diff_min"               : "Incremento mínimo del tiempo en obtención de información $(s)$",
                 "auction_info_time_diff_sum"               : "Sumatoria del incremento del tiempo en obtención de información $(s)$",
                 "auction_info_time_max"                    : "Tiempo máximo en obtención de información $(s)$", 
                 "auction_info_time_mean"                   : "Tiempo promedio en obtención de información $(s)$",
                 "auction_info_time_min"                    : "Tiempo mínimo en obtención de información $(s)$", 
                 "auction_info_time_sum"                    : "Sumatoria del tiempo en obtención de información $(s)$",
                 "erroneous_area"                           : "Area del mapa con errores $(m^2)$",
                 "exploration_cost"                         : "Distancia total recorrida por la flota $(m)$",
                 "exploration_efficiency"                   : "Eficiencia de exploración",
                 "exploration_time"                         : "Tiempo de exploración $(s)$",
                 "explored_area"                            : "Area explorada $(m^2)$",
                 "gvd_construction_time_diff_max"           : "Incremento máximo en construcción de GVD $(s)$", 
                 "gvd_construction_time_diff_mean"          : "Incremento promedio en construcción de GVD $(s)$",
                 "gvd_construction_time_diff_min"           : "Incremento mínimo en construcción de GVD $(s)$", 
                 "gvd_construction_time_diff_sum"           : "Sumatoria del Incremento en construcción de GVD $(s)$",
                 "gvd_construction_time_max"                : "Tiempo máximo en construcción de GVD $(s)$", 
                 "gvd_construction_time_mean"               : "Tiempo promedio en construcción de GVD $(s)$",
                 "gvd_construction_time_min"                : "Tiempo mínimo en construcción de GVD $(s)$", 
                 "gvd_construction_time_sum"                : "Sumatoria del tiempo en construcción de GVD $(s)$",
                 "gvd_update_auction_info_percent"          : "Porcentaje del tiempo de obtención de información en el que se construye el GVD $(\\%)$", 
                 "gvd_update_auction_info_percent_log_max"  : "Máximo porcentaje del tiempo de obtención de información en el que se construye el GVD $(\\%)$", 
                 "gvd_update_auction_info_percent_log_mean" : "Porcentaje promedio del tiempo de obtención de información en el que se construye el GVD $(\\%)$", 
                 "gvd_update_auction_info_percent_log_min"  : "Mínimo porcentaje del tiempo de obtención de información en el que se construye el GVD $(\\%)$", 
                 "gvd_update_auction_info_percent_log_sum"  : "Esto no tiene sentido $(\\%)$", 
                 "obj_id_time_diff_max"                     : "Incremento máximo en simplificación de fronteras $(s)$", 
                 "obj_id_time_diff_mean"                    : "Incremento promedio en simplificación de fronteras $(s)$",
                 "obj_id_time_diff_min"                     : "Incremento mínimo en simplificación de fronteras $(s)$", 
                 "obj_id_time_diff_sum"                     : "Sumatoria del Incremento en simplificación de fronteras $(s)$",
                 "obj_id_time_max"                          : "Tiempo máximo en simplificación de fronteras $(s)$", 
                 "obj_id_time_mean"                         : "Tiempo promedio en simplificación de fronteras $(s)$",
                 "obj_id_time_min"                          : "Tiempo mínimo en simplificación de fronteras $(s)$", 
                 "obj_id_time_sum"                          : "Sumatoria del tiempo en simplificación de fronteras $(s)$",
                 "obj_id_auction_info_percent"              : "Porcentaje del tiempo de obtención de información en el que se simplifican las fronteras (\\%)", 
                 "obj_id_auction_info_percent_log_max"      : "Máximo porcentaje del tiempo de obtención de información en el que se simplifican las fronteras $(\\%)$", 
                 "obj_id_auction_info_percent_log_mean"     : "Porcentaje promedio del tiempo de obtención de información en el que se simplifican las fronteras $(\\%)$", 
                 "obj_id_auction_info_percent_log_min"      : "Mínimo porcentaje del tiempo de obtención de información en el que se simplifican las fronteras $(\\%)$", 
                 "obj_id_auction_info_percent_log_sum"      : "Esto no tiene sentido $(\\%)$", 
                 "map_completeness"                         : "Completitud del mapa", 
                 "map_quality"                              : "Calidad del mapa"}
           
#############
# Functions #
#############
def cell_size_2_cells_per_meter(val):
    return round(math.pow(1/val,2))

def translate(val):
    return translations[val]

########
# MAIN #
########

# load config
if len(sys.argv) < 2 :
    print("table_gen.py [config]")
    sys.exit(1)

with open(sys.argv[1], 'r') as f:
    config = yaml.safe_load(f)


test_type = config['test_type']
mode      = config['mode']
data_dirs = config['data_dirs']
metrics   = config['metrics']
caption   = config['caption']
label     = config['label']
float_format = config.get('float_format',[])

extra_cols = 2
cols = len(metrics)+extra_cols

# start table generation

bs='\\'
br1='{'
br2='}'

output = f'''{bs}begin{br1}table{br2}[H]
%{datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
{bs}hbadness = 10000
{bs}tolerance=9999
{bs}emergencystretch=10pt
{bs}hyphenpenalty=10000
{bs}exhyphenpenalty=100
{bs}begin{br1}center{br2}

% {bs}begin{br1}adjustbox{br2}{br1}minipage=0.75{bs}paperwidth, center{br2}
{bs}begin{br1}adjustbox{br2}{br1}width=1{bs}textwidth{br2}
{bs}small

{bs}begin{br1}tabularx{br2}{br1}{bs}textwidth{br2}{br1}|X|C{br1}0.80cm{br2}|{"X|"*(cols-extra_cols)}{br2}

{bs}hline
'''

output+= f'{data_names[test_type][-1]} & {x_labels[mode]} & {" & ".join(list(map(translate,metrics)))} {bs}{bs} {bs}hline'

for i in range(0,len(data_dirs)):
    data_dir = data_dirs[i]
    data_name = os.path.basename(data_dir)
    with open(f"{data_dir_root}{data_dir}/data/digest.yaml", 'r') as f:
        digest = yaml.safe_load(f)

    output += "\\hline\n"
    output += f"{bs}multirow{br1}{len(xs[mode])}{br2}{br1}{bs}linewidth{br2}{br1}{bs}centering {data_names[test_type][i]}{br2}\n"

    for j in range(0,len(xs[mode])):
        x = xs[mode][j]
        if mode == 'celda':
            output += f'& {cell_size_2_cells_per_meter(x)}'
        elif mode == 'robot':
            output += f'& {x}'

        for k in range(0,len(metrics)):
            metric = metrics[k]
            if metric in digest:

                if metric in float_format:
                    mean_float_format     = float_format[metric].get('mean_f', default_mean_float_format)
                    mean_float_decimals   = float_format[metric].get('mean_d', default_mean_float_decimals)
                    stddev_float_format   = float_format[metric].get('stddev_f',default_stddev_float_format)
                    stddev_float_decimals = float_format[metric].get('stddev_d',default_stddev_float_decimals)
                elif 'default' in float_format:
                    mean_float_format     = float_format['default'].get('mean_f', default_mean_float_format)
                    mean_float_decimals   = float_format['default'].get('mean_d', default_mean_float_decimals)
                    stddev_float_format   = float_format['default'].get('stddev_f',default_stddev_float_format)
                    stddev_float_decimals = float_format['default'].get('stddev_d',default_stddev_float_decimals)
                else:
                    mean_float_format     = default_mean_float_format
                    mean_float_decimals   = default_mean_float_decimals
                    stddev_float_format   = default_stddev_float_format
                    stddev_float_decimals = default_stddev_float_decimals

                output += " & " + float2str(digest[metric]['mean'][x], mean_float_format,mean_float_decimals) + \
                            '±' + float2str(digest[metric]['std_dev'][x],stddev_float_format,stddev_float_decimals)
            else:
                output += " & 0 ± 0"

        if j != len(xs[mode])-1: # is not the last
            output += f"{bs}{bs} {bs}cline{br1}2-{cols}{br2}\n"
        else:
            output += f"{bs}{bs} {bs}hline"

output+=f'''
{bs}end{br1}tabularx{br2}
{bs}end{br1}adjustbox{br2}

{bs}caption{br1}{caption}{br2}
{bs}label{br1}{label}{br2}
{bs}end{br1}center{br2}

{bs}end{br1}table{br2}'''

print(output)

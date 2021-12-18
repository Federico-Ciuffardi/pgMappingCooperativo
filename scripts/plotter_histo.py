#!/usr/bin/env python3

# this was abandoned, the one used was plotter_histo_num.py 

import yaml
import sys
import os
import shutil
import json
import math
import pandas as pd
from collections import defaultdict
import PyGnuplot as pg # dependency
from PIL import Image
import time

########
# Vars #
########

colors = ['royalblue','dark-spring-green','orange-red','orange']

data_names = {
    "incrementalidad" : { 0 : "Incremental", 
                          1 : "No incremental"},

    "ident_obj"       : { 0 : "Simplificación de fronteras basada en cubrimiento", 
                          1 : "Simplificación de fronteras basada en K-Means",
                          2 : "Fronteras sin simplificar"},

    "desconocido"     : { 0 : "Desconocidas se consideran obstáculos", 
                          1 : "Desconocidas se consideran libres"},

    "planificacion"   : { 0 : "Jerárquica", 
                          1 : "Sobre el GVD"}
    }

x_labels = { "robot" : "Cantidad de robots", 
             "celda" : "Celdas por metro cuadrado"}

translations = { "auction_info_time_diff_max"               : "Incremento máximo del tiempo en obtención de información", 
                 "auction_info_time_diff_mean"              : "Incremento promedio del tiempo en obtención de información",
                 "auction_info_time_diff_min"               : "Incremento mínimo del tiempo en obtención de información",
                 "auction_info_time_diff_sum"               : "Sumatoria del incremento del tiempo en obtención de información",
                 "auction_info_time_max"                    : "Tiempo máximo en obtención de información", 
                 "auction_info_time_mean"                   : "Tiempo promedio en obtención de información",
                 "auction_info_time_min"                    : "Tiempo mínimo en obtención de información", 
                 "auction_info_time_sum"                    : "Sumatoria del tiempo en obtención de información",
                 "erroneous_area"                           : "Area del mapa con errores",
                 "exploration_cost"                         : "Costo de exploración",
                 "exploration_efficiency"                   : "Eficiencia de exploración",
                 "exploration_time"                         : "Tiempo de exploración",
                 "explored_area"                            : "Area explorada",
                 "gvd_construction_time_diff_max"           : "Incremento máximo en construcción de GVD", 
                 "gvd_construction_time_diff_mean"          : "Incremento promedio en construcción de GVD",
                 "gvd_construction_time_diff_min"           : "Incremento mínimo en construcción de GVD", 
                 "gvd_construction_time_diff_sum"           : "Sumatoria del Incremento en construcción de GVD",
                 "gvd_construction_time_max"                : "Tiempo máximo en construcción de GVD", 
                 "gvd_construction_time_mean"               : "Tiempo promedio en construcción de GVD",
                 "gvd_construction_time_min"                : "Tiempo mínimo en construcción de GVD", 
                 "gvd_construction_time_sum"                : "Tiempo del Incremento de obtención en información",
                 "gvd_update_auction_info_percent"          : "Porcentaje del tiempo de obtención de información en el que se construye el GVD (%)", 
                 "gvd_update_auction_info_percent_log_max"  : "Máximo porcentaje del tiempo de obtención de información en el que se construye el GVD", 
                 "gvd_update_auction_info_percent_log_mean" : "Porcentaje promedio del tiempo de obtención de información en el que se construye el GVD", 
                 "gvd_update_auction_info_percent_log_min"  : "Mínimo porcentaje del tiempo de obtención de información en el que se construye el GVD", 
                 "gvd_update_auction_info_percent_log_sum"  : "Esto no tiene sentido", 
                 "info_ob_mean"                             : "Tiempo promedio en identificación de objetivos",
                 "info_ob_percent_mean"                     : "Porcentaje promedio del tiempo de obtención de información en el que se identifican los objetivos",
                 "info_ob_sum"                              : "Sumatoria del tiempo en identificación de objetivos",
                 "map_completeness"                         : "Completitud del mapa", 
                 "map_quality"                              : "Calidad del mapa"}
           
histoOffset = { 2: ['($1-0.5)','($1+0.5)'],
                3: ['($1-0.75)','($1)','($1+0.75)']}
#############
# Functions #
#############

def cell_size_2_cells_per_meter(val):
    return round(math.pow(1/val,2))


########
# MAIN #
########

if len(sys.argv) < 4 :
    print("plotter_histo.py [output_dir] [test_type] [mode] [data_dir_1] [data_dir_2] ...")
    sys.exit(1)

output_dir = sys.argv[1] 

test_type = sys.argv[2] 

mode = sys.argv[3] 

data_dirs = []
for i in range(4,len(sys.argv)):
    data_dirs.append(sys.argv[i])

# if os.path.exists(output_dir):
#     print(f"{output_dir} already exists")
#     print(f"Continue?[y/N]:")
#     if sys.stdin.read(1) != 'y':
#         sys.exit(-1)

# os.mkdir(output_dir)

# Paso 1 cargar todas las pruebas de todos los data

tests = {}
for data_dir in data_dirs:
    with open(f"{data_dir}/data/digest.yaml", 'r') as f:
        data_name = os.path.basename(data_dir)
        digest = yaml.safe_load(f)
    for test_name, test_result in digest.items():
        tests.setdefault(test_name,{})
        tests[test_name][data_name] = test_result  

# Paso 2 armar graficas paralelas (?) para cada prueba 

for test_name, data_test_results in tests.items():
    pg.c( 'set term pngcairo font "arial,10" fontscale 1.0 size 1024, 768')
    pg.c(f'set output "{output_dir}/{test_name}.png"')

    pg.c(' set style data histogram')
    pg.c(' set style histogram cluster gap 1.5 errorbars lw 1')

    pg.c(' set style fill solid border rgb "black"')
    pg.c(' set boxwidth 0.5 absolute')

    pg.c(f'set ylabel "{translations[test_name]}"')
    pg.c( f'set xlabel "{x_labels[mode]}"')
    pg.c( 'set key tmargin')

    pg.c( 'set yrange [0:]')
    # if mode == "robot":
    #     pg.c( 'set xrange [0.5:5.5]')
    # elif mode == "celda":
    #     pg.c( 'set xrange [0.5:16.5]')

    # pg.c(" set offset graph 0.02,graph 0.02,0.02,0 ")
    # pg.c("set xtics right offset first 0.2")


    plot_cmd=""
    i = 0
    N=len(data_test_results)
    for data_name, test_results in data_test_results.items():
        xs = list(test_results['mean'].keys())
        if mode == "celda":
            xs = list(map(cell_size_2_cells_per_meter,xs))
        means = list(test_results['mean'].values())
        std_devs = list(test_results['std_dev'].values())

        if mode == "celda":
            xs = xs[::-1]
            means = means[::-1]
            std_devs = std_devs[::-1]

        dat_file_name = "/tmp/"+test_type+"-"+mode+"-"+test_name+"-"+data_name+".dat"
        pg.s([xs, means, std_devs],dat_file_name)

        if i!=0:
            plot_cmd += ", "

        plot_cmd+=f"'{dat_file_name}' using 2:3:xtic(1) title '{data_names[test_type][i]}' "
        # plot_cmd+=f"'{dat_file_name}' using {histoOffset[N][i]}:2:xtic(1) title '{data_names[test_type][i]}' with boxes lw 0.5 lc rgb '{colors[i]}', "+ \
        #           f"'{dat_file_name}' using {histoOffset[N][i]}:2:3:xtic(1) notitle with yerrorbars lc rgb 'black' pt 1 lw 1"

        i += 1

    pg.c("plot "+plot_cmd)
    pg.c( 'set term pngcairo font "arial,10" fontscale 1.0 size 1024, 768')
    pg.c(f'set output "{output_dir}/{test_name}.png"')
    pg.c('replot;')
    
# shutil.rmtree(tmp_dir)


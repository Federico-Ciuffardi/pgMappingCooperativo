#!/usr/bin/env python3

import yaml
import sys
import os
import shutil
import json
import math
import pandas as pd
from collections import defaultdict
from pygnuplot import gnuplot # dependency
from PIL import Image

########
# Vars #
########

# env vars
HOME=os.environ['HOME']

# config
## paths
pkg_dir=f"{HOME}/catkin_ws/src/pgmappingcooperativo"
data_dir=f"{pkg_dir}/data"
maps_dir=f"{pkg_dir}/maps"

## params 
x_param = "cell_size"
y_params = [ "exploration_time" ] # params on termination.yaml

#############
# Functions #
#############

def cell_size_2_cells_per_meter(val):
    return math.pow(1/val,2)

def cell_size_dict_2_cells_per_meter_dict(cell_size_dict):
    cells_per_meter_dict = {}
    for x_value, y_values in cell_size_dict.items():
        cells_per_meter_dict[cell_size_2_cells_per_meter(x_value)] = y_values
    return cells_per_meter_dict

def set_predigest_value(predigest, x_value , y_param, y_value):
    predigest.setdefault(y_param,{})
    predigest[y_param].setdefault(x_value,[])
    predigest[y_param][x_value].append(y_value)

def process_times(predigest, this_log_dir, x_value, name):
    times = []
    with open(f"{this_log_dir}/{name}", 'r') as f:
        lines_list = f.readlines()
        for line in lines_list:
            times.append(float(line.split()[1]))

    set_predigest_value(predigest, x_value, name+"_max", max(times))

    time_sum = sum(times)
    set_predigest_value(predigest, x_value, name+"_sum",  time_sum)

    set_predigest_value(predigest, x_value, name+"_mean", time_sum/len(times))

def isUnk(val):
    return val == 205

def cell_size_to_map_name(cell_size):
    map_name = ""
    for c in "{:.3f}".format(cell_size):
        if c == '.':
            map_name += '_'
        else:
            map_name += c 

    return map_name+"m.pgm"

def compare_maps(real_path, explored_path): 
    real_pixels     = list(Image.open(real_path).getdata())
    explored_pixels = list(Image.open(explored_path).getdata())
    
    if len(real_pixels) != len(explored_pixels):
        sys.exit(f"real map ({real_path}) and explored map ({explored_path}) dimensions do not match")

    erroneous_cells = 0
    real_map_cells = 0
    explored_map_cells = 0
    for (real_pixel, explored_pixel) in zip(real_pixels,explored_pixels):
        if isUnk(real_pixel): continue # skip if the real map has a value of unknown

        real_map_cells += 1
        if real_pixel != explored_pixel:
            erroneous_cells += 1

        if not isUnk(explored_pixel):
            explored_map_cells += 1

    return real_map_cells, explored_map_cells, erroneous_cells

########
# MAIN #
########

digest_dir = sys.argv[1] if len(sys.argv) > 1 else f"{data_dir}/latest"

log_dir = sys.argv[2] if len(sys.argv) > 2 else f"{pkg_dir}/log"

if os.path.exists(digest_dir):
    print(f"{digest_dir} already exists")
    print(f"Remove?[y/N]:")
    if sys.stdin.read(1) == 'y':
        shutil.rmtree(digest_dir)
    else:
        sys.exit(f"Please remove {digest_dir} or use another path")

os.mkdir(digest_dir)

# Paso 1: armar el los predigest
#   diccionario {key : predigest}
#   predigest debe tener info como para hacer el digest

predigest = {} 

for this_log in os.scandir(log_dir):
    # set the full path to the current log dir
    this_log_dir = f"{log_dir}/{this_log.name}"

    # check if file is valid
    if this_log.name == 'latest':
        continue
    if not os.path.isfile(f"{this_log_dir}/termination.yaml"):
        print(f"{this_log_dir} not terminated successfully")
        continue

    # load rosparams yaml
    with open(f"{this_log_dir}/rosparams.yaml", 'r') as f:
        rosparams_yaml = yaml.safe_load(f)

    # get the x_value
    x_value = rosparams_yaml[x_param]

    # load termination yaml
    with open(f"{this_log_dir}/termination.yaml", 'r') as f:
        termination_yaml = yaml.safe_load(f)

    # set the y_values (from termination.yaml) in the predigest 
    for y_param in y_params:
        set_predigest_value(predigest, x_value, y_param, termination_yaml[y_param])

    # set the distance sum in the predigest
    meters_traveled_list = []
    with open(f"{this_log_dir}/meters_traveled", 'r') as f:
        lines_list = f.readlines()
        for val in lines_list:
            meters_traveled_list.append(float(val))

    exploration_cost = sum(meters_traveled_list) # the sum of the distance travelled by the robots
    set_predigest_value(predigest, x_value, "exploration_cost", exploration_cost)

    # get map name
    map_name = rosparams_yaml['map_name']

    # get map info
    real_map_path     = f"{maps_dir}/{map_name}/{cell_size_to_map_name(x_value)}"
    explored_map_path = f"{this_log_dir}/map.pgm" 
    real_cells, explored_cells, erroneous_cells = compare_maps(real_map_path, explored_map_path)

    # set the explored_area in the predigest
    explored_area = explored_cells/cell_size_2_cells_per_meter(x_value) 
    set_predigest_value(predigest, x_value, "explored_area", explored_area)

    # set the exploration_efficiency in the predigest
    set_predigest_value(predigest, x_value, "exploration_efficiency", explored_area/exploration_cost)

    # set the erroneous_area in the predigest
    erroneous_area = erroneous_cells/cell_size_2_cells_per_meter(x_value)
    set_predigest_value(predigest, x_value, "erroneous_area", erroneous_area)

    # set the map_completeness in the predigest
    real_area = real_cells/cell_size_2_cells_per_meter(x_value)
    set_predigest_value(predigest, x_value, "map_completeness", explored_area/real_area)

    # set the map_quality in the predigest
    set_predigest_value(predigest, x_value, "map_quality", (explored_area-erroneous_area)/real_area)

    # set time related values in the predigest
    process_times(predigest, this_log_dir, x_value, 'auction_info_time')
    process_times(predigest, this_log_dir, x_value, 'auction_info_time_diff')

    process_times(predigest, this_log_dir, x_value, 'gvd_construction_time')
    process_times(predigest, this_log_dir, x_value, 'gvd_construction_time_diff')

# Paso 2 armar los digest de cada predigest

digest = {}
for y_param, values in predigest.items():
    digest[y_param] = {}

    for x_value, y_values in values.items():
        s1 = 0
        s2 = 0
        n = len(y_values)

        for y_value in y_values:
            s1 += y_value
            s2 += math.pow(y_value,2)

        digest[y_param].setdefault("samples",{})
        digest[y_param]["samples"][x_value] = n

        digest[y_param].setdefault("total",{})
        digest[y_param]["total"][x_value]   = s1

        digest[y_param].setdefault("mean",{})
        digest[y_param]["mean"][x_value]    = s1/n

        # https://math.stackexchange.com/questions/2148877/iterative-calculation-of-mean-and-standard-deviation
        digest[y_param].setdefault("std_dev",{})
        digest[y_param]["std_dev"][x_value] = math.sqrt(n*s2 - math.pow(s1,2))/n

# Paso 3 dejar el digest (quizas tambien el pre digest) en el file sistem

with open(f'{digest_dir}/predigest.yaml', 'w') as outfile:
    yaml.dump(predigest, outfile)

with open(f'{digest_dir}/digest.yaml', 'w') as outfile:
    yaml.dump(digest, outfile)

# Paso 4 post procesar el digest (hacer graficas u otro) 

for y_param, stats in digest.items():
    g = gnuplot.Gnuplot(terminal = 'pngcairo font "arial,10" fontscale 1.0 size 1024, 768',
                        output = f'"{digest_dir}/{y_param}.png"',
                        xlabel = f"\"cells per square meter\"",
                        ylabel = f"\"{y_param.replace('_',' ')}\"",
                        yrange = "[0:]")

    mean_df    = pd.DataFrame.from_dict(cell_size_dict_2_cells_per_meter_dict(stats["mean"]), orient='index')
    std_dev_df = pd.DataFrame.from_dict(cell_size_dict_2_cells_per_meter_dict(stats["std_dev"]), orient='index')
    
    df = mean_df.join(std_dev_df, lsuffix="mean", rsuffix="std_dev", sort = True)

    g.set("offset graph 0.02,graph 0.02,0.02,0 ")
    g.plot_data(df,'using 1:2 notitle with lines lw 0.5 lc rgb \'grey\'','using 1:2:3 notitle with yerrorbars lc rgb \'blue\' pt 1')

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


########
# Vars #
########

# env vars
HOME=os.environ['HOME']

# config
## paths
pkg_dir=f"{HOME}/catkin_ws/src/pgmappingcooperativo"
log_dir=f"{pkg_dir}/log"
data_dir=f"{pkg_dir}/data"

## params 
x_param = "cell_size"
y_params = [ "explored_cells" , "exploration_time" ] # params on termination.yaml

########
# MAIN #
########

digest_dir = sys.argv[1] if len(sys.argv) > 1 else "latest"
digest_dir = f"{data_dir}/{digest_dir}"

if os.path.exists(digest_dir):
    shutil.rmtree(digest_dir)

os.mkdir(digest_dir)

# Paso 1: armar el los predigest
#   diccionario {key : predigest}
#   predigest debe tener info como para hacer el digest

predigest = {} 

for this_log in os.scandir(log_dir):
    this_log_dir = f"{log_dir}/{this_log.name}"

    if(not os.path.isfile(f"{this_log_dir}/termination.yaml")):
        print(f"{this_log_dir} not terminated successfully")
        continue

    with open(f"{this_log_dir}/rosparams.yaml", 'r') as f:
        rosparams_yaml = yaml.safe_load(f)

    x_value = rosparams_yaml[x_param]

    with open(f"{this_log_dir}/termination.yaml", 'r') as f:
        termination_yaml = yaml.safe_load(f)

    for y_param in y_params:
        predigest.setdefault(y_param,{})
        predigest[y_param].setdefault(x_value,[])
        predigest[y_param][x_value].append(termination_yaml[y_param])


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
                        xlabel = f"\"{x_param.replace('_',' ')}\"",
                        ylabel = f"\"{y_param.replace('_',' ')}\"",
                        yrange = "[0:]")

    mean_df    = pd.DataFrame.from_dict(stats["mean"], orient='index')
    std_dev_df = pd.DataFrame.from_dict(stats["std_dev"], orient='index')
    
    df = mean_df.join(std_dev_df, lsuffix="mean", rsuffix="std_dev", sort = True)

    g.set("offset graph 0.02,graph 0.02,0,0 ")
    g.plot_data(df,'using 1:2 notitle with lines lw 0.5 lc rgb \'grey\'','using 1:2:3 notitle with yerrorbars lc rgb \'blue\' pt 1')

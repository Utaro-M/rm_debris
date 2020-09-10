#!/bin/sh

# これはコメントです

#./rh_plot.sh contact_70_JAXON_RED_2020_09-08_22-00_27のような感じで、引数としてファイルの名前を与える
echo $1
cd /home/utaro/matsuura@jsk.imi.i.u-tokyo.ac.jp/Documents/log/JAXON_RED
cd $1
# python ~/catkin_ws/jaxon_tutorial/src/log_plotter/src/log_plotter/datalogger_plotter_with_pyqtgraph.py -f contact_60_7_JAXON_RED_2020_09-08_22-03_44 --plot ~/research/rm_debris/log/rh_plot.yaml --layout ~/research/rm_debris/log/rh_layout.yaml

python ~/catkin_ws/jaxon_tutorial/src/log_plotter/src/log_plotter/datalogger_plotter_with_pyqtgraph.py -f $1 --plot ~/research/rm_debris/log/rh_plot.yaml --layout ~/research/rm_debris/log/rh_layout.yaml

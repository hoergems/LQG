import sys
import os
import subprocess
import glob

shared_path = os.path.dirname(os.path.abspath(__file__))

for i in xrange(1, 7):
    cmd = "python plot_stats.py -d ~/PhD/results_server/5_steps/random_scene/scene1/" + str(i) + " -s -c"
    popen = subprocess.Popen(cmd, cwd=shared_path, shell=True)
    popen.wait()
    
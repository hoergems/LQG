import sys
import os
import subprocess
import glob

shared_path = os.path.dirname(os.path.abspath(__file__))

for i in xrange(7, 13):
    cmd = "python plot_stats.py -d ~/PhD/results_server/5_steps/collision_aware/500_collision_penalty/" + str(i) + " -s"
    popen = subprocess.Popen(cmd, cwd=shared_path, shell=True)
    popen.wait()
    
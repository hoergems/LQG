import sys
import os
import subprocess
import glob

shared_path = os.path.dirname(os.path.abspath(__file__))

for i in xrange(1, 13):
    cmd = "cp LQG" + str(i) + "/config_hrf.yaml LQG" + str(i) + "/config_hrf.yamll"
    popen = subprocess.Popen(cmd, cwd=shared_path, shell=True)
    popen.wait()
    
    cmd = "cd LQG" + str(i) + " && git checkout -- config_hrf.yaml && git pull origin perf && cd .."
    popen = subprocess.Popen(cmd, cwd=shared_path, shell=True)
    popen.wait()
    
    cmd = "mv LQG" + str(i) + "/config_hrf.yamll LQG" + str(i) + "/config_hrf.yaml"
    popen = subprocess.Popen(cmd, cwd=shared_path, shell=True)
    popen.wait()
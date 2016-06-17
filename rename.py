import sys
import os
import subprocess
import glob

shared_path = os.path.dirname(os.path.abspath(__file__))
for i in xrange(6):
    path = shared_path + "/abt" + str(i + 1) + "/python/stats"
    files = glob.glob(path + "/log")
    for file in files:
        f = file.split("/")[-1].split("_")
        new_file = "log_abt_" + f[1]
        cmd = "mv " + str(file) + " " + str(path) + "/" + str(new_file)
        popen = subprocess.Popen(cmd, cwd=shared_path, shell=True)
        popen.wait()
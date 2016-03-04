import subprocess
import os
import sys

shared_path = os.path.dirname(os.path.abspath(__file__))
k = int(sys.argv[1])
k_end = int(sys.argv[2]) + 1

for i in xrange(k, k_end):
    cmd = "tar -zcvf hrf" + str(i) + ".tar.gz LQG" + str(i) + "/"
    popen = subprocess.Popen(cmd, cwd=shared_path, shell=True)
    popen.wait()
    
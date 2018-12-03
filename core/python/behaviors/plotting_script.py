import os
import pdb
import argparse
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()

parser.add_argument('--load_dir', default='./', type=str)
parser.add_argument('--save_dir', default='./', type=str)

args = parser.parse_args()

fp = open(os.path.join(args.load_dir, 'logger.txt'), 'r')
fp.readline()
fp.readline()
data = fp.read().split('\n')[:-1]

data_proc = []
for l in data:
    data_proc.append([float(w) for w in l.split(',')])

data = np.array(data_proc)
data = data[:3000]

t = data[:, 0]

ax = plt.subplot(3, 1, 1)
ax.plot(t, data[:, 1], 'r', label='Ws_t')
ax2 = plt.subplot(3, 1, 2)
ax2.plot(t, data[:, 2], 'b', label='Wa_t')
ax3 = plt.subplot(3, 1, 3)
ax3.plot(t, data[:, 3], 'g', label='GT')
ax.legend()
ax2.legend()
ax3.legend()

plt.savefig(os.path.join(args.save_dir, 'plot.png'))



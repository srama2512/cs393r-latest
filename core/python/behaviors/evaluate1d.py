import numpy as np
import pdb
import matplotlib
matplotlib.use('Agg')
from asami import ASAMI
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
import argparse
import os

def str2bool(v):
    return True if v.lower() in ['t', 'true', 'yes'] else False

parser = argparse.ArgumentParser()

parser.add_argument('--load_dir', default='./', type=str, required=True)
parser.add_argument('--save_dir', default='./', type=str, required=True)
parser.add_argument('--simFlag', default=True, type=str2bool, required=True)

args = parser.parse_args()

f = open(os.path.join(args.load_dir, 'smLogger.txt'), 'r')
data = f.read().split('\n')[:-1]
dimS = int(data[0])
data = data[1:]
sensorParams = np.array([[float(p) for p in d.split(',')] for d in data])
f.close()

f = open(os.path.join(args.load_dir, 'amLogger.txt'), 'r')
data = f.read().split('\n')[:-1]
dimA = int(data[0])
data = data[1:]
actionParams = np.array([[float(p) for p in d.split(',')] for d in data])
f.close()

f = open(os.path.join(args.load_dir, 'gtSensor.txt'), 'r')
data = f.read().split('\n')[:-1]
data = np.array([[float(p) for p in d.split(',')] for d in data])
obs = data[:, 0]
gtState = data[:, 1]

asami = ASAMI(dimA=dimA, dimS=dimS)

def findActionScalingFactor(pred, gt):
    # print(pred, gt)
    return np.dot(pred, gt) / (np.dot(pred, pred) + 1e-7)

action_error = np.zeros((actionParams.shape[0],))

if args.simFlag:
    actionCommands = np.linspace(0.2, 1.0, 9)
    gtActionVel = actionCommands * 240.0
else:
    f = open(os.path.join(args.load_dir, 'gtActuator.txt'), 'r')
    data = f.read().split('\n')[:-1]
    data = np.array([[float(p) for p in d.split(',')] for d in data])
    actionCommands = data[:, 0]
    gtActionVel = data[:, 1]
    actionCommands, gtActionVel = zip(*sorted(zip(actionCommands, gtActionVel), key=lambda x: x[0]))
    actionCommands, gtActionVel = np.array(actionCommands), np.array(gtActionVel)

fig= plt.figure(figsize=(6,15))

for i in range(0, actionParams.shape[0]):
    # initialise the models
    asami.At.beta = actionParams[i, 1:]
    asami.At.alpha = actionParams[i, 0]
    # Action model error
    predActionVel = asami.action_predict(actionCommands)
    scalingFactor = findActionScalingFactor(predActionVel, gtActionVel)
    # print(scalingFactor * predActionVel, gtActionVel)
    action_error[i] = np.sqrt(np.mean((gtActionVel - scalingFactor * predActionVel) ** 2))
    if i == actionParams.shape[0]-1:
        ax = plt.subplot(4, 1, 3)
        ax.plot(actionCommands, scalingFactor * predActionVel)
        ax.plot(actionCommands, gtActionVel, 'x')

# print(action_error)
axA = plt.subplot(4, 1, 1)
axA.set_ylim(0, 200)
axA.plot(np.arange(0, actionParams.shape[0]), action_error)
# plt.savefig('eval.png')

sensor_error = np.zeros((sensorParams.shape[0],))
for i in range(0, sensorParams.shape[0]):
    # initialise the models
    asami.St.beta = sensorParams[i, 1:]
    asami.St.alpha = sensorParams[i, 0]
    # sensor model error
    predState = asami.sensor_predict(obs)
    lr = LinearRegression().fit(predState[:, np.newaxis], gtState)
    
    predStateHat = lr.predict(predState[:, np.newaxis])
    # print(predStateHat, gtState)
    sensor_error[i] = np.sqrt(np.mean((gtState - predStateHat) ** 2))
    if i == sensorParams.shape[0]-1:
        ax = plt.subplot(4, 1, 4)
        ax.plot(obs, predStateHat)
        ax.plot(obs, gtState, linestyle='dashed')

axS = plt.subplot(4, 1, 2)
axS.set_ylim(0, 500)
axS.plot(np.arange(0, sensorParams.shape[0]), sensor_error)

plt.savefig(os.path.join(args.save_dir, 'eval.png'))

fig = plt.figure()
for i in range(0, sensorParams.shape[0]):
    # initialise the models
    asami.St.beta = sensorParams[i, 1:]
    asami.St.alpha = sensorParams[i, 0]
    # sensor model error
    predState = asami.sensor_predict(obs)
    # print(predStateHat, gtState)
    if i == sensorParams.shape[0]-1:
        plt.subplot(2, 1, 1)
        plt.plot(obs, predState)
        plt.subplot(2, 1, 2)
        plt.plot(obs, gtState, linestyle='dashed')

plt.savefig(os.path.join(args.save_dir, 'eval2.png'))

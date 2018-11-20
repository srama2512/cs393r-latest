import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pdb
import math
from sklearn.linear_model import LinearRegression

class IncrementalRegression(object):
	"""docstring for IncrementalRegression"""
	def __init__(self, gamma, dim):
		super(IncrementalRegression, self).__init__()
		self.gamma    = gamma
		self.dim      = dim
		self.V2SumMat = np.zeros((self.dim, self.dim))
		self.V1SumVec = np.zeros((self.dim,))
		self.VySumVec = np.zeros((self.dim,))
		self.ySum     = 0.0
		self.N        = 0.0
		self.beta     = np.zeros((self.dim,))
		self.alpha    = 0.0

	def update(self, x, y):
		""" x is a (d) size vector, y is a scalar """
		self.V2SumMat = (self.V2SumMat * self.gamma) + np.outer(x, x)
		self.V1SumVec = (self.V1SumVec * self.gamma) + x
		self.VySumVec = (self.VySumVec * self.gamma) + (x * y)
		self.ySum     = (self.ySum * self.gamma) + y
		self.N        = self.N * self.gamma + 1

	def computeParams(self):
		mtdm = self.V2SumMat - (np.outer(self.V1SumVec, self.V1SumVec) / self.N)
		mtdy = self.VySumVec - (self.V1SumVec * self.ySum) / self.N
		self.beta = np.dot(np.linalg.inv(mtdm), mtdy)
		self.alpha = (self.ySum - np.dot(self.V1SumVec, self.beta)) / self.N 

	def update_and_compute(self, *args):
		self.update(*args)
		self.computeParams()
		
	def predict(self, x):
		""" x is n x (d+1) matrix """
		return np.dot(x, self.beta) + self.alpha 

	@staticmethod
	def getPolyStack(x, d):
		data = [x**i for i in range(1, d+1)]
		return np.stack(data, axis=1)

if __name__ == '__main__':
	
	x = np.linspace(0.0, 2*math.pi, num=100)
	y = np.cos(x) + np.random.randn(*x.shape)*0.01

	dim = 5
	gamma = 1.0
	regressor = IncrementalRegression(gamma, dim)

	for xi, yi in zip(x,y):
		inputData = IncrementalRegression.getPolyStack(np.array([xi]), dim)
		regressor.update(inputData[0], yi)

	regressor.computeParams()
	yhat = regressor.predict(IncrementalRegression.getPolyStack(x, dim))

	sample_weights = [gamma**(100-i) for i in range(1,101)]
	lr = LinearRegression().fit(IncrementalRegression.getPolyStack(x, dim), y, sample_weights)

	plt.plot(x, y, linestyle='dashed')
	plt.plot(x, yhat)
	plt.plot(x, lr.predict(IncrementalRegression.getPolyStack(x, dim)))
	plt.savefig('plot.png')


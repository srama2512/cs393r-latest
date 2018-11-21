from regression import IncrementalRegression
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class ASAMI(object):
	"""docstring for ASAMI"""
	def __init__(self, nStart=100, gamma=0.99):
		super(ASAMI, self).__init__()
		self.nStart = nStart
		self.A0 = lambda c: 1.0 * c # Initial action model
		self.nUpdates = 0
		self.dimS = 3
		self.dimA = 3
		self.St = IncrementalRegression(gamma, self.dimS)
		self.At = IncrementalRegression(gamma, self.dimA)
		self.lambda_drift = 0.9
		self.Wa_t = 0.0
		self.Ws_t = 0.0
		self.pss = lambda x: IncrementalRegression.getPolyStack(x, self.dimS)
		self.psa = lambda x: IncrementalRegression.getPolyStack(x, self.dimA)
		self.c_t_stack_cumulative = np.zeros((self.dimA, ))

	def update(self, c_t, delta_t, obs_t=None):
		c_t_stack = self.psa(np.array([c_t]))
		self.c_t_stack_cumulative += c_t_stack[0] * delta_t

		self.Wa_t += self.A0(c_t) * delta_t
		# if self.nUpdates < 2*self.nStart:
		# 	self.Wa_t += self.A0(c_t) * delta_t
		# else:
		# 	self.Wa_t += self.At.predict(c_t_stack)[0] * delta_t

		if obs_t is not None:
			obs_t_stack = self.pss(np.array([obs_t]))
			if self.nUpdates > self.nStart:
				self.St.computeParams()
			# 	self.Ws_t = self.St.predict(obs_t_stack)[0]
			# 	self.At.update_and_compute(self.c_t_stack_cumulative, self.Ws_t)
			# 	self.Wa_t = (1 - self.lambda_drift) * self.Wa_t + self.lambda_drift * Ws_t
			print('x: {}, y: {}'.format(obs_t_stack[0], self.Wa_t))
			self.St.update(obs_t_stack[0], self.Wa_t)

		self.nUpdates += 1

	def sensor_predict(self, obs_t):
		# obs_t - n dimensional vector 
		# output - n dimensional vector
		obs_t_stack = self.pss(obs_t)
		return self.St.predict(obs_t_stack)

if __name__ == '__main__':
	obs_t = np.linspace(5.0, 40.0, 100)
	t = np.log(obs_t) + np.random.randn(*obs_t.shape) * 0.5
	asami = ASAMI(gamma=1.0)
	for i in range(1, 100):
		delta_t = t[i] - t[i-1]
		asami.update(1.0, delta_t, obs_t[i])
	asami.St.computeParams()
	t_pred = asami.sensor_predict(obs_t) + t[0]
	plt.plot(obs_t, t, linestyle='dashed')
	plt.plot(obs_t, t_pred)
	plt.savefig('plot.png')





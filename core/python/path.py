class Point2D:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def loc(self):
		return (self.x, self.y)

	def __str__(self):
		return '(' + str(self.x) + ', ' + str(self.y) + ')'

class Pose2D:
	def __init__(self, pos, t):
		self.pos = pos
		self.theta = t

	def loc(self):
		return (self.pos.x, self.pos.y)

	def __str__(self):
		return '(' + str(self.pos.x) + ', ' + str(self.pos.y) + ', ' + str(self.theta) + ')'

class Obstacle:
	def __init__(self, x, y, r):
		self.pos = Point2D(x, y)
		self.r = r

class PathPlanner:
	def __init__(self):
		self._obstacles = []
		self._target = None

	def update(self):
		pass
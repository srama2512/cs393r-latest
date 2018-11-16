from path import *
import cv2
import math
from random import randint as randi
from random import uniform as randu
import numpy as np
import pdb

class PathTester:
	def __init__(self, num_obstacles=2, max_x=600, max_y=450, border_size=50):
		self.max_x = max_x
		self.max_y = max_y
		self.border_size = border_size
		self.num_obstacles = num_obstacles

		self._robot_loc = Pose2D(self._get_random_pos(), randu(0, 2 * math.pi))
		self._obstacles = [self._get_random_pos() for i in range(num_obstacles)]
		self._target = self._get_random_pos()
		self._obstacle_radius = 30

		self.sim_img = None
		self.reset_canvas()

		self.C_ROBOT = (0, 0, 255)
		self.C_OBSTACLE = (0, 255, 0)
		self.C_TARGET = (255, 0, 0)

	def _get_random_pos(self):
		return Point2D(randi(self.border_size, self.max_x - self.border_size), \
			randi(self.border_size, self.max_y - self.border_size))

	def reset_canvas(self):
		self.sim_img = np.zeros((self.max_y, self.max_x, 3), np.uint8)

	def draw_robot(self):
		cv2.circle(self.sim_img, self._robot_loc.loc(), 5, self.C_ROBOT, -1)

	def draw_obstacles(self):
		for obs in self._obstacles:
			cv2.circle(self.sim_img, obs.loc(), self._obstacle_radius, self.C_OBSTACLE, 1)

	def draw_target(self):
		cv2.circle(self.sim_img, self._target.loc(), 5, self.C_TARGET, -1)

	def draw(self):
		self.reset_canvas()
		self.draw_robot()
		self.draw_obstacles()
		self.draw_target()

	def show(self):
		img = self.sim_img
		cv2.imshow('dst_rt', img)
		cv2.setMouseCallback('dst_rt', self.click_callback)

	def click_callback(self, event, x, y, flags, param):
		
		def inside_obstacle(x, y, obs):
			dist = math.sqrt((x - obs.x) ** 2 + (y - obs.y) ** 2)
			if dist < self._obstacle_radius:
				return True
			else:
				return False

		if event == cv2.EVENT_MBUTTONDOWN:
			self._target = Point2D(x, y)

		elif event == cv2.EVENT_LBUTTONDOWN:
			new_obstacles = [obs for obs in self._obstacles if not inside_obstacle(x, y, obs)]
			if len(new_obstacles) != self.num_obstacles:
				self.num_obstacles = len(new_obstacles)
				self._obstacles = new_obstacles
			else:
				self.num_obstacles += 1
				self._obstacles.append(Point2D(x, y))

		else:
			pass

	def __str__(self):
		ret = ''
		ret += 'RPos: ' + self._robot_loc.__str__() + '\n'
		ret += 'Obstacles:\n'
		ret += '\n'.join([obs.__str__() for obs in self._obstacles])
		ret += '\n'
		ret += 'Target: ' + self._target.__str__() + '\n'

		return ret



p = PathTester()

while True:
	p.draw()
	print(p)
	p.show()
	c = cv2.waitKey(10)

	if c == ord('q') or c == 27:
		break


from path import *
import cv2
import math
from random import randint as randi
from random import uniform as randu
import numpy as np
import pdb
import argparse

class PathTester:
	def __init__(self, num_obstacles=2, max_x=600, max_y=450, border_size=50):
		self.max_x = max_x
		self.max_y = max_y
		self.border_size = border_size

		self._robot_loc = Pose2D(self._get_random_pos(), randu(0, 2 * math.pi))
		self._obstacle_radius = 30
		self._obstacles = [Obstacle(*self._get_random_pos().loc(), r=self._obstacle_radius) for i in range(num_obstacles)]
		self._target = self._get_random_pos()
		self._validate_obstacles()

		self.sim_img = None
		self.reset_canvas()

		self.C_ROBOT = (0, 0, 255) # RED
		self.C_OBSTACLE = (0, 255, 0) # GREEN
		self.C_TARGET = (255, 0, 0) # BLUE
		self.C_PATH = (255, 255, 255) # WHITE

	def _get_random_pos(self):
		return Point2D(randi(self.border_size, self.max_x - self.border_size), \
			randi(self.border_size, self.max_y - self.border_size))

	def reset_canvas(self):
		self.sim_img = np.zeros((self.max_y, self.max_x, 3), np.uint8)

	def draw_robot(self):
		cv2.circle(self.sim_img, self._robot_loc.loc(), 5, self.C_ROBOT, -1)

	def draw_obstacles(self):
		for obs in self._obstacles:
			cv2.circle(self.sim_img, (int(obs.pos.x), int(obs.pos.y)), int(obs.r), self.C_OBSTACLE, 1)

	def draw_target(self):
		cv2.circle(self.sim_img, self._target.loc(), 5, self.C_TARGET, -1)

	def draw_path(self, path):
		for p in path:
			if p[2] == 'line':
				x1 = int(p[0].x + self._robot_loc.pos.x)
				y1 = int(p[0].y + self._robot_loc.pos.y)
				x2 = int(p[1].x + self._robot_loc.pos.x)
				y2 = int(p[1].y + self._robot_loc.pos.y)
				cv2.line(self.sim_img, (x1, y1), (x2, y2), self.C_PATH, 1)

	def draw(self):
		self.reset_canvas()
		self.draw_robot()
		self.draw_obstacles()
		self.draw_target()

	def show(self):
		img = self.sim_img
		cv2.imshow('dst_rt', img)
		cv2.setMouseCallback('dst_rt', self.click_callback)

	def _validate_obstacles(self):
		self._obstacles = merge_obstacles(self._obstacles)
		# new_obstacles = [obs for obs in self._obstacles if not (obs._is_inside(self._robot_loc.pos) or obs._is_inside(self._target))]

	def click_callback(self, event, x, y, flags, param):

		if event == cv2.EVENT_MBUTTONDOWN:
			self._target = Point2D(x, y)

		elif event == cv2.EVENT_LBUTTONDOWN:
			new_obstacles = [obs for obs in self._obstacles if not obs._is_inside(Point2D(x, y))]
			if len(new_obstacles) != len(self._obstacles):
				self._obstacles = new_obstacles
			else:
				self._obstacles.append(Obstacle(x, y, self._obstacle_radius))

			self._validate_obstacles()

		else:
			pass

	def get_obstacles_egocentric(self):
		self._validate_obstacles()
		obstacles = [Obstacle(obs.pos.x - self._robot_loc.pos.x, obs.pos.y - self._robot_loc.pos.y, \
			obs.r) for obs in self._obstacles]
		return obstacles

	def get_target_egocentric(self):
		return Point2D(self._target.x - self._robot_loc.pos.x, self._target.y - self._robot_loc.pos.y)

	def get_keypoints_egocentric(self):
		return self.get_obstacles_egocentric(), self.get_target_egocentric()

	def __str__(self):
		ret = ''
		ret += 'RPos: ' + self._robot_loc.__str__() + '\n'
		ret += 'Obstacles:\n'
		ret += '\n'.join([obs.pos.__str__() + ' r: ' + str(obs.r) for obs in self._obstacles])
		ret += '\n'
		ret += 'Target: ' + self._target.__str__() + '\n'

		return ret

parser = argparse.ArgumentParser()
parser.add_argument('--planner', default='geo', type=str, help='planner type to use')
args = parser.parse_args()

p = PathTester()
p._robot_loc = Pose2D(Point2D(494, 357), 2.8743314216643725)
p._obstacles = [Obstacle(288, 183, 30), Obstacle(311, 297, 30), Obstacle(430, 305, 30)]
p._target = Point2D(196, 151)

# p._robot_loc = Pose2D(Point2D(0, 100), 0)
# p._obstacles = [Obstacle(172, 114, 30), Obstacle(135, 113, 30), Obstacle(164, 90, 30), Obstacle(207, 134, 30)]
# p._target = Point2D(269, 111)
# p._obstacles = [Obstacle(0, 0, 30), Obstacle(50, 0, 30), Obstacle(25, 60, 30)]

if args.planner == 'apm':
	planner = PotentialPathPlanner()
elif args.planner == 'geo':
	planner = GeometricPathPlanner()
elif args.planner == 'rrt':
	planner = RRTPlanner()
else:
	raise NotImplementedError('unknown planner type')

while True:
	p.draw()
	print(p)
	# TODO: change this
	path = planner.update(*p.get_keypoints_egocentric())
	print([(pt1.__str__(), pt2.__str__(), etype) for pt1, pt2, etype in path])
	p.draw_path(path)
	p.show()
	c = cv2.waitKey(10)

	if c == ord('q') or c == 27:
		break
	elif c == ord('d'):
		pdb.set_trace()


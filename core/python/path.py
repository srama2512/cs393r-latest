import pdb
import math
import time
import heapq
import numpy as np
from numpy.random import uniform as randu
from circle_utils import minidisk

class Point2D:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def loc(self):
		return (self.x, self.y)

	def dist(self, pt):
		return math.sqrt((self.x - pt.x) ** 2 + (self.y - pt.y) ** 2)

	def pt_in_dir(self, pt, d):
		ux = pt.x - self.x
		uy = pt.y - self.y
		mag = math.sqrt(ux ** 2 + uy ** 2)
		assert mag != 0.0
		ux /= mag
		uy /= mag

		return Point2D(self.x + d * ux, self.y + d * uy)

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
		self.pos = Point2D(float(x), float(y))
		self.r = float(r)

	def _is_overlapping(self, obs):
		dist = math.sqrt((self.pos.x - obs.pos.x) ** 2 + (self.pos.y - obs.pos.y) ** 2)
		return dist < self.r + obs.r

	def _is_inside(self, pt):
		dist = math.sqrt((self.pos.x - pt.x) ** 2 + (self.pos.y - pt.y) ** 2)
		return dist < self.r

	def _merge_together(self, obs):
		dist = math.sqrt((self.pos.x - obs.pos.x) ** 2 + (self.pos.y - obs.pos.y) ** 2)
		self.pos = Point2D((self.pos.x + obs.pos.x) / 2.0, (self.pos.y + obs.pos.y) / 2.0)
		self.r = (self.r + obs.r + dist) / 2.0

def merge_n_obs_together(obstacles):
	centers = np.array([[obs.pos.x, obs.pos.y] for obs in obstacles])
	m_center, r = minidisk(centers)
	max_r = max([obs.r for obs in obstacles])

	return Obstacle(m_center[0], m_center[1], r + max_r)

def merge_obstacle_helper(tent_obs, clusters, max_cid):
	if len(tent_obs) == 1:
		return clusters

	ids = list(tent_obs.keys())
	for i in range(len(ids)):
		for j in range(i + 1, len(ids)):
			cid1, cid2 = ids[i], ids[j]
			if tent_obs[cid1]._is_overlapping(tent_obs[cid2]):
				all_cluster = clusters[cid1] + clusters[cid2]
				del clusters[cid1]
				del tent_obs[cid1]
				del clusters[cid2]
				del tent_obs[cid2]
				tent_obs[max_cid] = merge_n_obs_together(all_cluster)
				clusters[max_cid] = all_cluster
				max_cid += 1

				return merge_obstacle_helper(tent_obs, clusters, max_cid)

	return clusters

def merge_obstacles(obstacles):

	clusters = {k: [obs] for k, obs in enumerate(obstacles)}
	tent_obs = {k: obs for k, obs in enumerate(obstacles)}
	max_cid = len(obstacles)

	clusters = merge_obstacle_helper(tent_obs, clusters, max_cid)

	ret_obstacles = []
	for cid in clusters.keys():
		ret_obstacles.append(merge_n_obs_together(clusters[cid]))

	return ret_obstacles

class Line2D:
	def __init__(self, p1, p2):
		self.pt1 = p1
		self.pt2 = p2

	def cost(self):
		return self.pt1.dist(self.pt2)

	def _intersect_with_circle(self, obs):
		a = (self.pt2.x - self.pt1.x) ** 2 + (self.pt2.y - self.pt1.y) ** 2
		b = 2. * ((self.pt1.x - obs.pos.x) * (self.pt2.x - self.pt1.x) + \
			(self.pt1.y - obs.pos.y) * (self.pt2.y - self.pt1.y))
		c = (self.pt1.x - obs.pos.x) ** 2 + (self.pt1.y - obs.pos.y) ** 2 - (obs.r ** 2)

		discrim = b * b - 4. * a * c

		if discrim < 0:
			return False

		else:
			discrim = math.sqrt(discrim)

			t1 = (-b - discrim) / (2. * a)
			t2 = (-b + discrim) / (2. * a)

			if t1 >= 0 and t1 <= 1:
				return True
			elif t2 >= 0 and t2 <= 1:
				return True

			return False

def sqr_dist_point2D(pt1, pt2):
	return ((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2)

def dist_point2D(pt1, pt2):
	return sqr_dist_point2D(pt1, pt2) ** 0.5

class PathPlanner(object):
	def __init__(self):
		self._obstacles = []
		self._target = None
		self._path = None

		self._num_frames = 0
		self._last_planned = -float('inf')
		self._refresh_rate = 1

		self._num_plans = 0
		self._total_time_spent_planning = 0.

	def _update_plan_time(self, t):
		self._num_plans += 1
		self._total_time_spent_planning += t

	def get_average_time(self):
		if self._num_plans == 0:
			return 0.
		return self._total_time_spent_planning / float(self._num_plans)

	def update(self, obstacles, target):
		pass

	def _is_path_valid(self):
		self._num_frames += 1
		if self._num_frames - self._last_planned < self._refresh_rate:
			return True
		self._last_planned = self._num_frames
		return False

class PotentialPathPlanner(PathPlanner):
	def __init__(self):
		super(PotentialPathPlanner, self).__init__()
		self._obstacles = []
		self._target = None
		self._stepsize = 1000.
		self._goal_eps = 10.
		self._k_attr = 1.0
		self._k_rep = 1000000.0
		self.q_star_mult = 3.
		self.max_path_len = 20

	def distance_to_target(self, curr_pos, target):
		return dist_point2D(curr_pos, target)

	def compute_target_potential_grad(self, curr_pos, target):
		potential = dist_point2D(curr_pos, target)
		theta = math.atan2(target.y - curr_pos.y, target.x - curr_pos.x)
		return potential * math.cos(theta), potential * math.sin(theta)

	def compute_obstacle_potential_grad(self, curr_pos, obstacle):
		dist = dist_point2D(curr_pos, obstacle.pos)
		q_star = self.q_star_mult * obstacle.r
		if (dist > q_star):
			return 0., 0.
		potential =  (1. / (q_star) - 1. / dist)
		theta = math.atan2(obstacle.pos.y - curr_pos.y, obstacle.pos.x - curr_pos.x)
		return potential * math.cos(theta), potential * math.sin(theta)

	def get_new_position(self, curr_pos, potential_x, potential_y):
		new_x = curr_pos.x + self._stepsize * potential_x
		new_y = curr_pos.y + self._stepsize * potential_y
		return Point2D(new_x, new_y)

	def compute_potential_gradient(self, curr_pos, target, obstacles):
		grad_x = 0.
		grad_y = 0.
		grad_target_x, grad_target_y = self.compute_target_potential_grad(curr_pos, target)
		grad_x += self._k_attr * grad_target_x
		grad_y += self._k_attr * grad_target_y
		for obs in obstacles:
			grad_obs_x, grad_obs_y = self.compute_obstacle_potential_grad(curr_pos, obs)
			grad_x += self._k_rep * grad_obs_x
			grad_y += self._k_rep * grad_obs_y
		grad_norm = np.sqrt(grad_x ** 2 + grad_y ** 2)

		return grad_x / grad_norm, grad_y / grad_norm

	def update(self, obstacles, target):
		if self._is_path_valid():
			return self._path

		if target is None:
			return None

		st_time = time.time()
		self._path = []
		curr_pos = Point2D(0., 0.)
		dist = self.distance_to_target(curr_pos, target)
		n_obstacles = len(obstacles)
		while True:
			if len(self._path) > self.max_path_len:
				break
			dist = self.distance_to_target(curr_pos, target)
			if (dist < self._goal_eps):
				break
			grad_x, grad_y = self.compute_potential_gradient(curr_pos, target, obstacles)
			new_pos = self.get_new_position(curr_pos, grad_x, grad_y)
			self._path.append((curr_pos, new_pos, 'line'))
			curr_pos = new_pos
		time_to_plan = time.time() - st_time
		self._update_plan_time(time_to_plan)

		return self._path

class GeometricPathPlanner(PathPlanner):

	class VisibilityGraph:
		class Edge:
			def __init__(self, i1, i2, pt1, pt2, cost, etype):
				self.i1 = i1
				self.i2 = i2
				self.pt1 = pt1
				self.pt2 = pt2
				self.etype = etype
				self.cost = cost
		def __init__(self):
			self._nvertex = 2
			self._graph = {}
			self.ARC_COST_MULTIPLIER = 1.2

		def add_edge(self, i1, i2, pt1, pt2, cost, etype):
			if i1 not in self._graph:
				self._graph[i1] = []
			if i2 not in self._graph:
				self._graph[i2] = []
			self._graph[i1].append(self.Edge(i1, i2, pt1, pt2, cost, etype))
			self._graph[i2].append(self.Edge(i2, i1, pt2, pt1, cost, etype))

		def dijkstras(self):
			min_dist = [float('inf') if i != 0 else 0.0 for i in range(self._nvertex)]
			visited = [False for i in range(self._nvertex)]
			back_pointers = [None for i in range(self._nvertex)]
			pr_queue = []
			heapq.heappush(pr_queue, (0., 0))

			while len(pr_queue) > 0:
				cost, idx = heapq.heappop(pr_queue)

				if visited[idx]:
					continue
				visited[idx] = True

				for edge in self._graph[idx]:
					ecost = edge.cost * (1.0 if edge.etype == 'line' else self.ARC_COST_MULTIPLIER)
					nidx = edge.i2
					if min_dist[nidx] > cost + ecost:
						min_dist[nidx] = cost + ecost
						back_pointers[nidx] = (idx, (edge.pt1, edge.pt2, edge.etype))
						heapq.heappush(pr_queue, (cost + ecost, nidx))

			path = []
			idx = 1
			while idx != 0:
				path.append(back_pointers[idx][1])
				idx = back_pointers[idx][0]

			path.reverse()
			return path

	def __init__(self):
		super(GeometricPathPlanner, self).__init__()
		self._obstacles = []
		self._target = None
		self._vis_graph = self.VisibilityGraph()

	def _get_tangent_pts(self, obs, pt):
		dx = obs.pos.x - pt.x
		dy = obs.pos.y - pt.y
		dd = math.sqrt(dx ** 2 + dy ** 2)
		r = obs.r

		if r / dd < 1.0:
			a = math.asin(r / dd)
		else:
			raise ValueError('Cannot calculate tangent from a point within the circle')
		b = math.atan2(dy, dx)

		t = b - a
		ta = Point2D(obs.pos.x + r * math.sin(t), obs.pos.y + r * -math.cos(t))
		t = b + a
		tb = Point2D(obs.pos.x + r * -math.sin(t), obs.pos.y + r * math.cos(t))

		return ta, tb

	def _get_arc_length(self, pt1, pt2, r):
		chord_len = pt1.dist(pt2)
		angle = 2. * math.asin(chord_len / (2. * r))
		return angle * r

	def _update_visibility_graph(self, src, tgt, i1=0, i2=1, ignore_obs=[]):
		print(i1, i2, ignore_obs)
		mline = Line2D(src, tgt)
		intersecting_obs = []
		for i, obs in enumerate(self._obstacles):
			if i not in ignore_obs and mline._intersect_with_circle(obs):
				intersecting_obs.append((i, obs))

		if len(intersecting_obs) == 0:	# straight line path available
			cost = src.dist(tgt)
			# updating the vis graph
			self._vis_graph.add_edge(i1, i2, src, tgt, cost, 'line')
		else:
			intersecting_obs.sort(key=lambda obs_tup: src.dist(obs_tup[1].pos))
			o1, o2 = self._get_tangent_pts(intersecting_obs[0][1], src)
			o3, o4 = self._get_tangent_pts(intersecting_obs[0][1], tgt)
			r = intersecting_obs[0][1].r
			if self._get_arc_length(o1, o3, r) > self._get_arc_length(o1, o4, r):
				o4, o3 = o3, o4
			io1 = self._vis_graph._nvertex
			io2 = self._vis_graph._nvertex + 1
			io3 = self._vis_graph._nvertex + 2
			io4 = self._vis_graph._nvertex + 3
			self._vis_graph._nvertex += 4

			# updating the vis graph
			self._vis_graph.add_edge(io1, io3, o1, o3, self._get_arc_length(o1, o3, r), 'arc')
			self._vis_graph.add_edge(io2, io4, o2, o4, self._get_arc_length(o2, o4, r), 'arc')
			self._update_visibility_graph(src, o1, i1, io1, ignore_obs=ignore_obs + [intersecting_obs[0][0]])
			self._update_visibility_graph(src, o2, i1, io2, ignore_obs=ignore_obs + [intersecting_obs[0][0]])
			self._update_visibility_graph(o3, tgt, io3, i2, ignore_obs=ignore_obs + [intersecting_obs[0][0]])
			self._update_visibility_graph(o4, tgt, io4, i2, ignore_obs=ignore_obs + [intersecting_obs[0][0]])

	def update(self, obstacles, target):
		if self._is_path_valid():
			return self._path

		if target is None:
			return None
		self._obstacles = merge_obstacles(obstacles)
		self._target = target
		self._vis_graph = self.VisibilityGraph()
		if len([o for o in self._obstacles if o._is_inside(self._target)]) > 0:
			print ('\n\n\n Warning: Target inside obstacle circle \n\n\n')
			return self._path
		if len([o for o in self._obstacles if o._is_inside(Point2D(0., 0.))]) > 0:
			print ('\n\n\n Warning: Source inside obstacle circle \n\n\n')
			return self._path
		try:
			st_time = time.time()
			self._update_visibility_graph(Point2D(0, 0), self._target)
			self._path = self._vis_graph.dijkstras()
			time_to_plan = time.time() - st_time
			self._update_plan_time(time_to_plan)
		except:
			print ('\n\n\n Warning: Cannot calculate tangent from a point within the circle \n\n\n')

		return self._path

class RRTPathPlanner(PathPlanner):

	class RRT:
		def __init__(self):
			self._nodes = {0: Point2D(0., 0.)}
			self._parent_ptr = {0: None}
			self._max_id = 1

		def _get_closest_node(self, pt):
			nearest_neighs = [k for k, v in sorted(self._nodes.items(), key=lambda kv: kv[1].dist(pt))]
			return nearest_neighs[0]

		def _add_node(self, pt, nearest):
			key = self._max_id
			self._max_id += 1
			self._nodes[key] = pt
			self._parent_ptr[key] = nearest

			return key

		def _get_path(self, src, target):
			path = []

			while target != src:
				parent = self._parent_ptr[target]
				path.append((self._nodes[parent], self._nodes[target], 'line'))
				target = parent

			path.reverse()
			return path

	def __init__(self):
		super(RRTPathPlanner, self).__init__()
		self._obstacles = []
		self._target = None
		self._delta_q = 100.
		self._target_bias = 0.05
		self._tree = self.RRT()

		self._xmin, self._xmax = -1000.0, 3000.0
		self._ymin, self._ymax = -2000.0, 2000.0
		self._bound_tolerance = 0.1

		self._max_tree_size = 200
		self._target_tolerance = 20.

	def _get_search_bounds(self):
		points = [Point2D(0., 0.), self._target] + [obs.pos for obs in self._obstacles]
		self._xmin = min([pt.x for pt in points])
		self._xmax = max([pt.x for pt in points])
		self._ymin = min([pt.y for pt in points])
		self._ymax = max([pt.y for pt in points])

		xspan = self._xmax - self._xmin
		self._xmin -= xspan * self._bound_tolerance
		self._xmax += xspan * self._bound_tolerance
		yspan = self._ymax - self._ymin
		self._ymin -= yspan * self._bound_tolerance
		self._ymax += yspan * self._bound_tolerance


	def _add_node_to_tree_multiple(self, q_rand):
		q_near_key = self._tree._get_closest_node(q_rand)
		q_near = self._tree._nodes[q_near_key]
		obs_interrupt = False

		while q_near.dist(q_rand) > self._delta_q:
			q_new = q_near.pt_in_dir(q_rand, self._delta_q)
			line = Line2D(q_near, q_new)
			overlap = [obs for obs in self._obstacles if line._intersect_with_circle(obs)]
			if len(overlap) > 0:
				obs_interrupt = True
				break
			# pdb.set_trace()
			q_near_key = self._tree._add_node(q_new, q_near_key)
			q_near = self._tree._nodes[q_near_key]

		if obs_interrupt:
			return

		line = Line2D(q_near, q_rand)
		overlap = [obs for obs in self._obstacles if line._intersect_with_circle(obs)]
		if len(overlap) > 0:
			return
		self._tree._add_node(q_rand, q_near_key)

	def _add_node_to_tree_single(self, q_rand):
		q_near_key = self._tree._get_closest_node(q_rand)
		q_near = self._tree._nodes[q_near_key]
		if q_near.dist(q_rand) < self._delta_q:
			q_new = q_rand
		else:
			q_new = q_near.pt_in_dir(q_rand, self._delta_q)

		line = Line2D(q_near, q_new)
		overlap = [obs for obs in self._obstacles if line._intersect_with_circle(obs)]
		if len(overlap) > 0:
			return
		self._tree._add_node(q_new, q_near_key)

	def _get_random_point(self):
		return Point2D(randu(self._xmin, self._xmax), randu(self._ymin, self._ymax))

	def _plan(self):
		self._tree = self.RRT()
		self._get_search_bounds()

		while self._tree._max_id < self._max_tree_size:
			t_near_key = self._tree._get_closest_node(self._target)
			t_near = self._tree._nodes[t_near_key]
			if t_near.dist(self._target) < self._target_tolerance:
				break

			if randu(0., 1.) < self._target_bias:
				q_rand = self._target
			else:
				q_rand = self._get_random_point()
			self._add_node_to_tree_single(q_rand)

		src = 0
		target = self._tree._get_closest_node(self._target)

		return self._tree._get_path(src, target)

	def update(self, obstacles, target):
		if target is None:
			return None
		self._obstacles = obstacles
		self._target = target
		self._path = self._plan()

		return self._path

class RRTStarPathPlanner(PathPlanner):

	class RRT:
		def __init__(self):
			self._nodes = {0: Point2D(0., 0.)}
			self._parent_ptr = {0: None}
			self._costs = {0: 0}
			self._max_id = 1

		def _get_closest_node(self, pt):
			nearest_neighs = [k for k, v in sorted(self._nodes.items(), key=lambda kv: kv[1].dist(pt))]
			return nearest_neighs[0]

		def _get_closest_nodes_in_radius(self, pt, radius):
			near = [k for k, v in self._nodes.items() if v.dist(pt) <= radius]
			return near

		def _add_node(self, pt, nearest_key):
			key = self._max_id
			self._max_id += 1
			self._nodes[key] = pt
			self._parent_ptr[key] = nearest_key
			self._costs[key] = self._costs[nearest_key] + pt.dist(self._nodes[nearest_key])

			return key

		def _move_parent(self, q_near_key, q_new_key):
			self._parent_ptr[q_near_key] = q_new_key
			self._costs[q_near_key] = self._costs[q_new_key] + \
				self._nodes[q_near_key].dist(self._nodes[q_new_key])

		def _get_path(self, src, target):
			path = []

			while target != src:
				parent = self._parent_ptr[target]
				path.append((self._nodes[parent], self._nodes[target], 'line'))
				target = parent

			path.reverse()
			return path

		def num_nodes(self):
			return len(self._nodes)

		def cost(self, key):
			return self._costs[key]

	def __init__(self):
		super(RRTStarPathPlanner, self).__init__()
		self._obstacles = []
		self._target = None
		self._delta_q = 200.
		self._target_bias = 0.05
		self._tree = self.RRT()

		self._xmin, self._xmax = -1000.0, 3000.0
		self._ymin, self._ymax = -2000.0, 2000.0
		self._bound_tolerance = 0.1

		self._max_tree_size = 50
		self._max_samples = 100
		self._target_tolerance = 200.
		self._gamma = 10000

	def _get_search_bounds(self):
		points = [Point2D(0., 0.), self._target] + [obs.pos for obs in self._obstacles]
		self._xmin = min([pt.x for pt in points])
		self._xmax = max([pt.x for pt in points])
		self._ymin = min([pt.y for pt in points])
		self._ymax = max([pt.y for pt in points])

		xspan = self._xmax - self._xmin
		self._xmin -= xspan * self._bound_tolerance
		self._xmax += xspan * self._bound_tolerance
		yspan = self._ymax - self._ymin
		self._ymin -= yspan * self._bound_tolerance
		self._ymax += yspan * self._bound_tolerance

	def _is_overlapping(self, line):
		overlap = [obs for obs in self._obstacles if line._intersect_with_circle(obs)]
		return len(overlap) > 0

	def _inside_obstacles(self, pt):
		obstacles = [obs for obs in self._obstacles if obs._is_inside(pt)]
		return len(obstacles) > 0

	def _add_node_to_tree_single(self, q_rand):
		q_nearest_key = self._tree._get_closest_node(q_rand)
		q_nearest = self._tree._nodes[q_nearest_key]
		n = self._tree.num_nodes()
		radius = min(math.sqrt(self._gamma * math.log(n) / n), self._delta_q)

		if q_nearest.dist(q_rand) < self._delta_q:
			q_new = q_rand
		else:
			q_new = q_nearest.pt_in_dir(q_rand, self._delta_q)

		q_nears = self._tree._get_closest_nodes_in_radius(q_new, radius)

		line = Line2D(q_nearest, q_new)

		if not self._is_overlapping(line):

			q_min_key = q_nearest_key
			cost_min = self._tree.cost(q_min_key) + line.cost()

			for q_near_key in q_nears:
				line_ = Line2D(self._tree._nodes[q_near_key], q_new)
				if self._is_overlapping(line_):
					continue
				c_ = self._tree.cost(q_near_key) + line_.cost()
				if c_ < cost_min:
					q_min_key = q_near_key
					cost_min = c_

			q_new_key = self._tree._add_node(q_new, q_min_key)
			q_nears = [q for q in q_nears if q != q_min_key]

			for q_near_key in q_nears:
				line_ = Line2D(q_new, self._tree._nodes[q_near_key])
				if self._is_overlapping(line_):
					continue
				if self._tree.cost(q_near_key) > self._tree.cost(q_new_key) + line_.cost():
					self._tree._move_parent(q_near_key, q_new_key)

	def _is_old_path_valid(self):
		if self._path is None or len(self._path) == 0:
			print ('\n\n Path empty\n\n')
			return False

		for l in self._path:
			line_ = Line2D(l[0], l[1])
			if len([o for o in self._obstacles if line_._intersect_with_circle(o)]) > 0:
				print ('\n\n Path IS intersecting\n\n')
				return False

		print ('\n\n Path is NOT intersecting\n\n')
		return True

	def _get_random_point(self):
		return Point2D(randu(self._xmin, self._xmax), randu(self._ymin, self._ymax))

	def _plan(self):
		self._tree = self.RRT()
		self._get_search_bounds()
		sample_cnt = 0

		while self._tree._max_id < self._max_tree_size and sample_cnt < self._max_samples:
			t_near_key = self._tree._get_closest_node(self._target)
			t_near = self._tree._nodes[t_near_key]
			if t_near.dist(self._target) < self._target_tolerance:
				break

			if randu(0., 1.) < self._target_bias:
				q_rand = self._target
			else:
				q_rand = self._get_random_point()
			self._add_node_to_tree_single(q_rand)
			sample_cnt += 1

		src = 0
		target = self._tree._get_closest_node(self._target)

		path = self._tree._get_path(src, target)

		return path

	def _simplify_path(self, path):
		if path is None or len(path) < 2:
			return path

		src = path[0][0]

		for i, line in enumerate(path):
			if i == 0:
				continue
			shortcut = Line2D(src, line[1])
			if self._is_overlapping(shortcut):
				ret_path = [(src, path[i - 1][1], 'line')] + path[i:]
				return ret_path

		return [(src, path[-1][1], 'line')]

	def update(self, obstacles, target):
		if self._is_path_valid():
			return self._path

		if target is None:
			return None
		self._obstacles = obstacles
		self._target = target
		st_time = time.time()
		self._path = self._plan()
		self._path = self._simplify_path(self._path)
		time_to_plan = time.time() - st_time
		self._update_plan_time(time_to_plan)

		return self._path

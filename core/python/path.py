import pdb
import math
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

	ids = tent_obs.keys()
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

	def update(self, obstacles, target):
		pass

class PotentialPathPlanner(PathPlanner):
	def __init__(self):
		super(PotentialPathPlanner, self).__init__()
		self._obstacles = []
		self._target = None
		self._stepsize = 0.1
		self._path = None
		self._goal_eps = 10.
		self._k_attr = 1.0
		self._k_rep = 10000.0
		self.q_star_mult = 5.
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
		return grad_x, grad_y

	def update(self, obstacles, target):
		if target is None:
			return None
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
		self._path = None

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
		if target is None:
			return None
		self._obstacles = merge_obstacles(obstacles)
		self._target = target
		self._vis_graph = self.VisibilityGraph()
		try:
			self._update_visibility_graph(Point2D(0, 0), self._target)
			self._path = self._vis_graph.dijkstras()
		except:
			print ('\n\n\n Warning: Cannot calculate tangent from a point within the circle \n\n\n')

		return self._path

class RRTPlanner(PathPlanner):

	class RRT:
		def __init__(self):
			self._nodes = {0: Point2D(0., 0.)}
			self._tree = {0: []}
			self._max_id = 1

		def _get_closest_node(self, pt):
			nearest_neighs = [k for k, v in sorted(self._nodes.iteritems(), key=lambda kv: kv[1].dist(pt))]
			return nearest_neighs[0]

		def _add_node(self, pt, nearest):
			key = self._max_id
			self._max_id += 1
			self._nodes[key] = pt
			self._tree[key] = []
			self._tree[nearest].append(key)
			self._tree[key].append(nearest)

			return key

		def _dijkstras(self, src, target):
			min_dist = [float('inf') if i != src else 0.0 for i in range(self._max_id)]
			visited = [False for i in range(self._max_id)]
			back_pointers = [None for i in range(self._max_id)]
			pr_queue = []
			heapq.heappush(pr_queue, (0., src))

			while len(pr_queue) > 0:
				cost, idx = heapq.heappop(pr_queue)

				if visited[idx]:
					continue
				visited[idx] = True

				for neigh in self._tree[idx]:
					ecost = self._nodes[neigh].dist(self._nodes[idx])
					if min_dist[neigh] > cost + ecost:
						min_dist[neigh] = cost + ecost
						back_pointers[neigh] = (idx, (self._nodes[idx], self._nodes[neigh], 'line'))
						heapq.heappush(pr_queue, (cost + ecost, neigh))

			path = []
			idx = target
			while idx != 0:
				path.append(back_pointers[idx][1])
				idx = back_pointers[idx][0]

			path.reverse()
			return path

	def __init__(self):
		super(RRTPlanner, self).__init__()
		self._obstacles = []
		self._target = None
		self._path = None
		self._delta_q = 20.
		self._tree = self.RRT()

		self._xmin, self._xmax = -1000.0, 3000.0
		self._ymin, self._ymax = -2000.0, 2000.0

		self._max_tree_size = 200
		self._target_tolerance = 20.

	def _get_search_bounds(self):
		points = [Point2D(0., 0.), self._target] + [obs.pos for obs in self._obstacles]
		self._xmin = min([pt.x for pt in points])
		self._xmax = max([pt.x for pt in points])
		self._ymin = min([pt.y for pt in points])
		self._ymax = max([pt.y for pt in points])

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

			q_rand = self._get_random_point()
			self._add_node_to_tree_single(q_rand)

		src = 0
		target = self._tree._get_closest_node(self._target)

		return self._tree._dijkstras(src, target)

	def update(self, obstacles, target):
		if target is None:
			return None
		self._obstacles = obstacles
		self._target = target
		self._path = self._plan()

		return self._path

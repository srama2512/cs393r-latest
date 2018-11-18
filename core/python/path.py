import pdb
import math
import heapq

class Point2D:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def loc(self):
		return (self.x, self.y)

	def dist(self, pt):
		return math.sqrt((self.x - pt.x) ** 2 + (self.y - pt.y) ** 2)

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
		self.r = max(self.r, obs.r) + dist / 2.0

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

class PathPlanner:
	def __init__(self):
		self._obstacles = []
		self._target = None

	def update(self):
		pass

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
		self._obstacles = obstacles
		self._target = target
		self._vis_graph = self.VisibilityGraph()
		try:
			self._update_visibility_graph(Point2D(0, 0), self._target)
			self._path = self._vis_graph.dijkstras()
		except:
			print ('Warning: Cannot calculate tangent from a point within the circle')

		return self._path


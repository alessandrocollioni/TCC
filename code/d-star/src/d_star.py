import sys
import math
from math import sqrt
#import d_star as Dstar


class Graph:
    def __init__(self, map, width, height):
        self.weights = {}
        self.width = width
        self.height = height
        self.walls = []
        self.setObstacles(map, height, width)
        self.cost = {}
        self.MapCellCOST_UNWALKABLE = float("inf")

    def costCell(self, Cell, value = None):
    	if value == None:
    		if not(Cell in self.cost):
    			return self.MapCellCOST_UNWALKABLE

    	else:
    		if not(Cell in self.cost):
    			self.cost[Cell] = value
    	return self.cost[Cell]
    	

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def setObstacles(self, img, height, width):
        x = 0
        y = 0
        while ( x < height ):
            self.walls.append([])
            while ( y < width ):
            	#print x, ", ", y
            	if (img[y][x] == 0):
            		self.walls[x].append(True)
                else:   
                    self.walls[x].append(False)
                y = y + 1
            x = x + 1
            y = 0

        '''for x in img:
            l = []
            for y in x:
                if y != 255:
                    l.append(True)
                else:           
                    l.append(False)
            self.walls.append(l)'''

    def passable(self, id):
        (x, y) = id
        if self.walls[x][y] == False or self.walls[x][y] == 0:
            return None
        return id
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)]

        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


class Planner:

	def __init__ (self, map, start, goal):
		'''constantes'''
		self.MAX_STEPS = 1000000
		self.Math_INF = float("inf")

		''' Clear lists'''
		self._open_list_TC = []
		self._open_hash = {}
		self._cell_hash = {}
		self._path = []
		self.lastInsertCellHash = None

		self._km = 0;

		self._map = map;
		self._start = start;
		self._goal = goal;
		self._last = self._start;

		self._rhs(self._goal, 0.0);

		self._list_insert(self._goal, (self._h(self._start, self._goal), 0))

		'''fazer funcoes: _h, _rhs, _list_insert'''

	def path(self):
		return self._path;

	def goal(self, Cell_u = None):

		if (Cell_u != None):
			self._goal = Cell_u;

		return self._goal;

	def replan(self):
	
		self._path = []
		
		result = self._compute();
		
		'''Couldn't find a solution'''
		if  result == False :
			return False
		print "path", self._path
		current = self._start;
		self._path.append(current);

		'''Follow the path with the least cost until goal is reached'''
		while (current != self._goal):
		
			if (current == None):
				return False;
			current = self._min_succ(current)[0];

			self._path.append(current);

		return True;
		'''funcao _compute, _min_succ'''

	def start(self, Cell_u = None):

		if (Cell_u != None):
			self._start = Cell_u;

		return self._start;


	

	def update(self, Cell_u, cost):
		if (Cell_u == self._goal):
			return;

		# Update km
		self._km += self._h(self._last, self._start);
		self._last = self._start;

		self._cell(Cell_u);

		cost_old = self._map.costCell(Cell_u);
		cost_new = cost;
		self._map.costCell(Cell_u, cost);


		

		tmp_cost_old = None
		tmp_cost_new = None
		tmp_rhs = None
		tmp_g = None

		# Update Cell_u
		for nbrs in self._map.neighbors(Cell_u) :

			if (nbrs != None):

				self._map.costCell(Cell_u, cost_old)
				tmp_cost_old = self._cost(Cell_u, nbrs);
				self._map.costCell(Cell_u, cost_new)
				tmp_cost_new = self._cost(Cell_u, nbrs);

				tmp_rhs = self._rhs(Cell_u);
				tmp_g = self._g(nbrs);

				if (tmp_cost_old > tmp_cost_new):

					if (Cell_u != self._goal):

						self._rhs(Cell_u, min(tmp_rhs, (tmp_cost_new + tmp_g)));

				else:
					if (tmp_rhs == (tmp_cost_old + tmp_g)):

						if (Cell_u != self._goal):

							self._rhs(Cell_u, self._min_succ(Cell_u)[1]);

		self._update(Cell_u);

		# Update neighbors
		for  nbrs in self._map.neighbors(Cell_u):

			if (nbrs != None):

				self._map.costCell(Cell_u, cost_old)
				tmp_cost_old = self._cost(Cell_u, nbrs);
				self._map.costCell(Cell_u, cost_new)
				tmp_cost_new = self._cost(Cell_u, nbrs);

				tmp_rhs = self._rhs(nbrs);
				tmp_g = self._g(Cell_u);

				if (tmp_cost_old > tmp_cost_new):

					if (nbrs != self._goal):

						self._rhs(nbrs[i], min(tmp_rhs, (tmp_cost_new + tmp_g)));

				else:
					if (tmp_rhs == (tmp_cost_old + tmp_g)):

						if (nbrs != self._goal):

							self._rhs(nbrs, self._min_succ(nbrs)[1]);

				self._update(nbrs);
	
	def _cell(self, Cell_u):

		if self.lastInsertCellHash != None :
			if Cell_u in self._cell_hash:
				if self._cell_hash[Cell_u] != self.lastInsertCellHash :
					return;

		h = self.Math_INF;
		self._cell_hash[Cell_u] = (h, h);
		self.lastInsertCellHash = (h, h);

	def _compute(self):

		if (len(self._open_list_TC) == 0):
			return False;

		'''KeyCompare key_compare;'''

		attempts = 0;

		'''Map::Cell* u;
		pair<double,double> k_old;
		pair<double,double> k_new;
		Map::Cell** nbrs;
		double g_old;
		double tmp_g, tmp_rhs;'''

		#print "DEBUG: _open_list_TC->",self._open_list_TC[0][0][0]
		print "DEBUG: _open_list_TC->",self._open_list_TC
		for i, j in enumerate(self._open_list_TC):
		    if i == 0:
		        openList_first = j
		        
		#print "DEBUG: _open_list_TC[0]->",self._open_list_TC[0]
		#print "DEBUG: _open_list_TC[0][0]->",self._open_list_TC[0][0]
		#print "DEBUG: _open_list_TC[0][0][0]->",self._open_list_TC[0][0][0]
		#print "DEBUG: _open_list_TC[0][0][1]->",self._open_list_TC[0][0][1]
		#print "DEBUG: _open_list_TC[0][1]->",self._open_list_TC[0][1]
		#print "DEBUG: _open_list_TC[0][1][0]->",self._open_list_TC[0][1][0]
		#print "DEBUG: _open_list_TC[0][1][1]->",self._open_list_TC[0][1][1]
		#self._open_list_TC[0][1] = int(self._open_list_TC[0][1])
		#print int(self._open_list_TC[0][1])
		#valKeyCompare = self._KeyCompare((openList_first[0]), self._k(self._start));
		#print "DEBUG: not(len(_open_list_TC))->",not(len(self._open_list_TC) == 0)
		#print "DEBUG: valKeyCompare->", valKeyCompare
		#print "DEBUG: rhs->",self._rhs(self._start) 
		#print "DEBUG: g->",self._g(self._start)
		#print "DEBUG: rhs != g->",(self._rhs(self._start) != self._g(self._start))
		#print "DEBUG: while->",(( not(len(self._open_list_TC) == 0) and valKeyCompare) or (self._rhs(self._start) != self._g(self._start)))
		while (( not(len(self._open_list_TC) == 0) and self._KeyCompare((openList_first[0]), self._k(self._start))) or (self._rhs(self._start) != self._g(self._start))):
			print "DEBUG: len not empity->", not(len(self._open_list_TC) == 0), "| valKeyCompare->", self._KeyCompare((openList_first[0]), self._k(self._start)), "| rhs+g->", (self._rhs(self._start) != self._g(self._start))
			#print "DEBUG: _open_list_TC->",self._open_list_TC, len(self._open_list_TC)
			'''Reached max steps, quit'''
			if (++attempts > self.MAX_STEPS):
				return False;
                
			Cell_u = openList_first[1]
            #print "DEBUG: cellU->", Cell_u;
            
			k_old = openList_first[0];
			k_new = self._k(Cell_u);

			tmp_rhs = self._rhs(Cell_u);
			tmp_g = self._g(Cell_u);
			
			if (self._KeyCompare(k_old, k_new)):
				self._list_update(Cell_u, k_new);
			else :
				if (tmp_g > tmp_rhs) :

					print "aquii", Cell_u, tmp_rhs
					self._g(Cell_u, tmp_rhs)
					tmp_g = tmp_rhs

					self._list_remove(Cell_u);

					for neighbor in self._map.neighbors(Cell_u) :

						if (neighbor != None):

							#adicionado
							#if (neighbor != self._goal):

							self._rhs(neighbor, min(self._rhs(neighbor), self._cost(neighbor, Cell_u) + tmp_g));
							

							self._update(neighbor);
				else :
				
					g_old = tmp_g;
					self._g(Cell_u, self.Math_INF);

					'''Perform action for Cell_u'''
					'''if (Cell_u != self._goal):
					abs
						self._rhs(Cell_u, self._min_succ(Cell_u)[1]);
					'''

					self._update(Cell_u);

					nbrs = self._map.neighbors(Cell_u)

					for neighbor in nbrs+[Cell_u] :

				
						if (neighbor != None):
						
							if self._rhs(neighbor) == (self._cost(neighbor, Cell_u) + g_old):
							
								if (neighbor != self._goal):
								
									self._rhs(neighbor, self._min_succ(neighbor)[1]);

							self._update(neighbor);

			#organiza o valor para o key compare
			for i, j in enumerate(self._open_list_TC):
			    if i == 0:
			        openList_first = j	
			        #print"DEBUG: j value->", j	
			#print "DEBUG: _open_list_TC->",self._open_list_TC
		return True;
	

	'''funcao _k, _g, _list_remove, _cost, _update'''

	def _cost(self,  Cell_a, Cell_b):
		#if self._map.costCell(Cell_a) == self._map.MapCellCOST_UNWALKABLE or self._map.costCell(Cell_b) == self._map.MapCellCOST_UNWALKABLE :
		if self._map.costCell(Cell_a) == self._map.MapCellCOST_UNWALKABLE:
			self._map.costCell(Cell_a, 0)
		if self._map.costCell(Cell_b) == self._map.MapCellCOST_UNWALKABLE :
			self._map.costCell(Cell_b, 0)
			print "return", True
			#return self._map.MapCellCOST_UNWALKABLE;

		print "return", False
		
		dx = math.fabs(Cell_a[0] - Cell_b[0])
		dy = math.fabs(Cell_a[1] - Cell_b[1])
		scale = 1.0;

		if ((dx + dy) > 1):
			scale = sqrt(2);

		print "scale", scale * ((self._map.costCell(Cell_a) + self._map.costCell(Cell_b)) / 2);

		#print "DEBUG: _open_list_TC->",self._open_list_TC, len(self._open_list_TC)
		return scale * ((self._map.costCell(Cell_a) + self._map.costCell(Cell_b)) / 2);

	def _g(self, Cell_u, value=None):
		self._cell(Cell_u);
		g_rhs = self._cell_hash[Cell_u];

		if value != None :
			aux = (value, g_rhs[1]);
			self._cell_hash[Cell_u] = aux;
			g_rhs = aux

		return g_rhs[0];

	def _h(self, Cell_a, Cell_b ):
		min = math.fabs((Cell_a[0] - Cell_b[0]))
		max = math.fabs((Cell_a[1] - Cell_b[1]))
		tmp = 0;
		
		if min > max :
			tmp = min;
			min = max;
			max = tmp;
		
		
		return ((sqrt(2) - 1.0) * min + max);	

	def _k (self, Cell_u):

		g = self._g(Cell_u);
		rhs = self._rhs(Cell_u);
		if (g < rhs) :
			min = g
		else : 
			min =rhs
		return ((min + self._h(self._start, Cell_u) + self._km), min);

	def _list_insert(self, Cell_u, tuplaDouble_k):
		print "DEBUG: _list_insert->", Cell_u, tuplaDouble_k
		val = (tuplaDouble_k, Cell_u);
		self._open_list_TC.append(val);
		self._open_hash[Cell_u] = self._open_list_TC.index(val);

	def _list_remove(self, Cell_u):
		print "DEBUG: _list_remove->", Cell_u
		#print "DEBUG: _open_list_TC->", self._open_list_TC
        #print "DEBUG: _open_hash[Cell_u]->", self._open_hash[Cell_u];
		if Cell_u in self._open_hash :
			for k, v in self._open_hash.iteritems():
				if v > self._open_hash[Cell_u] :
					self._open_hash[k] -= 1
			del self._open_list_TC[self._open_hash[Cell_u]];
			del self._open_hash[Cell_u];

	def _list_update(self, Cell_u, tuple_k):
		print "DEBUG: _list_update->", Cell_u
		pos1 = self._open_hash[Cell_u];
		pos2 = pos1;
		size = len(self._open_list_TC);
		if (pos1 == None):
			pos1 = 0
			pos2 = 0

		if (pos1 == size):
			pos2 = size;
		else:
			print pos2
			pos2 += 1;

		del self._open_list_TC[pos1];
		
		print "DEBUG: inserindo in _open_list_TC->", pos2, (tuple_k, Cell_u );
		self._open_list_TC.insert(pos2, (tuple_k, Cell_u ));
		#self._open_list_TC.index((tuple_k, Cell_u ))
		self._open_hash[Cell_u] = pos2
		print "DEBUG: agora tenho no _open_list_TC->", self._open_list_TC;


	def _min_succ(self, Cell_u):
		
		min_cell = None;
		min_cost = self.Math_INF;


		for neighbor in self._map.neighbors(Cell_u) :		

			if (neighbor != None):

				tmp_cost = self._cost(Cell_u,neighbor);
				tmp_g = self._g(neighbor);

				if (tmp_cost == self.Math_INF or tmp_g == self.Math_INF):
					print "continuou", tmp_cost, tmp_g, min_cost, min_cell, neighbor
					continue;
				tmp_cost += tmp_g;

				if (tmp_cost < min_cost):

					min_cell = neighbor;
					min_cost = tmp_cost;

		return (min_cell, min_cost)
	
			
	def _rhs(self, Cell_u,  value=None):

		#substitui
		#if (Cell_u == self._goal):
		if (Cell_u == self._start):
			return 0;

		self._cell(Cell_u);
		g_rhs = self._cell_hash[Cell_u];
		#print 'DEBUG: value->', value
		if (value != None):
			aux = (g_rhs[0], value);
			self._cell_hash[Cell_u] = aux;
			g_rhs = aux
			g_rhs = aux
		#print 'DEBUG: rhs value->', g_rhs[1]

		
		return g_rhs[1];
		'''fazer funcoes: _cell, pair(o q eh?)'''

	def _update(self, Cell_u):

		diff = self._g(Cell_u) != self._rhs(Cell_u);
		exists = Cell_u in self._open_hash;

		if (diff and exists):
			self._list_update(Cell_u, self._k(Cell_u));
		
		else:
			if (diff and not(exists)):
		
				self._list_insert(Cell_u, self._k(Cell_u));
		
			else:
				if ( not(diff) and exists):
					self._list_remove(Cell_u);
			
	def _KeyCompare(self, p1, p2):
		'''p1 e p2 sao uma tupla de dubles'''
		print "(DEBUG: p1 e p2->",p1, p2, "end)"
		if ( p1[0] < p2[0] ):
			return True;
		else:
			if (p1[0]> p2[0]):
				return False;
			else:
				if ( p1[1] < p2[1]):
					return True;
				else:
					if (p1[1] > p2[1]):
						return False;
		return False;

def main():
	
	matrixMap = [
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0]
	]
	'''matrixMap[5][3] = 1
	matrixMap[5][4] = 1
	matrixMap[5][5] = 1
	matrixMap[5][6] = 1
	matrixMap[5][7] = 1
	matrixMap[5][8] = 1'''
	
	graph = Graph(matrixMap, 10,10)

	print "neighbors", graph.neighbors((0,1))

	planner = Planner(graph, (0,0), (2,2))
 
 	#print planner.update((5,5), 10);
 	#print planner.goal();
 	#print planner.start();
	print "DEBUG: replan->", planner.replan()

	print(planner.path())



if __name__ == "__main__" :
	main()


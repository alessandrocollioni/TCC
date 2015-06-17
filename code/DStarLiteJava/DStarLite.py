'''*
 *
 * @author daniel beard
 * http:#danielbeard.wordpress.com
 * http:#github.com/paintstripper
 *
 * Copyright (C) 2012 Daniel Beard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
'''
import State as clsState
import Pair as clsPair
import math
#from math import sqrt


class CellInfo():

	def __init__(self):
		self.g=0;
		self.rhs=0;
		self.cost=0;


class ipoint2():


	#default constructor
	def __init__(self, x = None, y = None):
	
		self.x = x;
		self.y = y;
	




class DStarLite():


	#Default constructor
	def __init__(self):
		#print "__init__"

		#self.Member variables
		self.path = [];
		#self.C1;
		self.k_m = 0;
		self.s_start = clsState.State.fromNone();
		self.s_goal  = clsState.State.fromNone();
		self.s_last  = clsState.State.fromNone();
		#self.maxSteps;
		#self.openList = new PriorityQueue<State>();
		self.openList = []
		self.openListCount = 0

		#Change back to private****
		self.cellHash = {};
		self.openHash = {};

		#Constants
		self.M_SQRT2 = math.sqrt(2.0);
	
		self.maxSteps = 800000;
		self.C1	= 1;
	

	'''
	 * Initialise Method
	 * @params start and goal coordinates
	'''
	def init( self, sX, sY, gX, gY):
		#print "init",sX, sY, gX, gY
	
		self.cellHash.clear();
		self.path = []
		self.openHash.clear();
		while((len(self.openList) != 0)): 
			del self.openList[0]

		self.k_m = 0;

		self.s_start.x = sX;
		self.s_start.y = sY;
		self.s_goal.x  = gX;
		self.s_goal.y  = gY;

		tmp = CellInfo();
		tmp.g   = 0;
		tmp.rhs = 0;
		tmp.cost = self.C1;

		self.cellHash[self.s_goal] = tmp;

		tmp = CellInfo();
		tmp.g = tmp.rhs = self._heuristic(self.s_start,self.s_goal);
		tmp.cost = self.C1;
		self.cellHash[self.s_start] = tmp;
		self.s_start = self._calculateKey(self.s_start);

		self.s_last = self.s_start;

	

	'''
	 * CalculateKey(state u)
	 * As per [S. Koenig, 2002]
	'''
	def _calculateKey(self, u):
		#print "calculateKey",u.x, u.y
	
		val = min(self._getRHS(u), self._getG(u));

		u.k.setFirst((val + self._heuristic(u,self.s_start) + self.k_m));
		u.k.setSecond(val);
		
		return u;
	

	'''
	 * Returns the rhs value for state u.
	'''
	def _getRHS( self, u):
		#print "getRHS",u.x, u.y
	
		if (u.eq(self.s_goal)):
			#print "return value RHS", 0
			return 0;

		#if the cellHash doesn't contain the State u

		for k, v in self.cellHash.iteritems():
			if k.x == u.x and k.y == u.y:
				#print "return rhs RHS", v.rhs
				return v.rhs;
		aux = self._heuristic(u, self.s_goal);
		#print "return aux RHS", aux
		return aux

		'''if (cellHash.get(u) == null)
			return heuristic(u, s_goal);
		return cellHash.get(u).rhs;'''


	'''
	 * Returns the g value for the state u.
	'''
	def _getG( self, u):
		#print "getG",u.x, u.y
	
		#if the cellHash doesn't contain the State u
		for k, v in self.cellHash.iteritems():
			if k.x == u.x and k.y == u.y:
				#print "return G VALOR", v.g
				return v.g;
		aux = self._heuristic(u,self.s_goal);
		#print "return aux G", aux
		return aux
		'''if (cellHash.get(u) == null)
			return heuristic(u,s_goal);
		return cellHash.get(u).g;'''


	'''
	 * Pretty self explanatory, the self._heuristic we use is the 8-way distance
	 * scaled by a constant C1 (should be set to <= min cost)
	'''
	def _heuristic( self, a,  b):
		#print "heuristic",a.x, a.y, b.x, b.y
	
		return self._eightCondist(a,b)*self.C1;
	

	'''
	 * Returns the 8-way distance between state a and state b
	'''
	def _eightCondist( self, a,  b):
		#print "eightCondist",a.x, a.y, b.x, b.y
	
		temp = None;
		min = math.fabs(a.x - b.x);
		max = math.fabs(a.y - b.y);
		if (min > max):
		
			temp = min;
			min = max;
			max = temp;
		
		return ((self.M_SQRT2-1.0)*min + max);

	

	def replan(self):
		print "replan"
	
		self.path = []

		res = self._computeShortestPath();

		print "end from computeShortestPath"
		if (res < 0):
		
			print("No Path to Goal");
			return False;
		

		n = [];
		cur = self.s_start;

		if (self._getG(self.s_start) == float("inf")):
		
			print("No Path to Goal");
			return False;
		

		while (cur.neq(self.s_goal)):
		
			self.path.append(cur);
			n = [];
			n = self._getSucc(cur);

			if (len(n) == 0):
			
				print ("No Path to Goal");
				return False;

			

			cmin = float("inf");
			tmin = 0;   
			smin = clsState.State.fromNone();

			for i in n:
				#self.openListCount = self.openListCount + 1;
			
				val  = self._cost(cur, i);
				val2 = self._trueDist(i,self.s_goal) + self._trueDist(self.s_start, i);
				val += self._getG(i);

				if (self._close(val,cmin)) :
					if (tmin > val2) :
						tmin = val2;
						cmin = val;
						smin = i;
						self.openListCount = self.openListCount + 1;
					
				else:
					if (val < cmin) :
						tmin = val2;
						cmin = val;
						smin = i;
						self.openListCount = self.openListCount + 1;
				
			
			n = []
			cur = clsState.State.fromState(smin);
			#cur = smin;
		
		self.path.append(self.s_goal);
		return True;
	

	'''
	 * As per [S. Koenig,2002] except for two main modifications:
	 * 1. We stop planning after a number of steps, 'maxsteps' we do this
	 *    because this algorithm can plan forever if the start is surrounded  by obstacles
	 * 2. We lazily remove states from the open list so we never have to iterate through it.
	'''
	def _calculateKey2(self, u):
		self.s_start = self._calculateKey(u)
		return self.s_start

	def _computeShortestPath(self):
		print "computeShortestPath"
	
		s = [];

		if (len(self.openList) == 0) :
			return 1;

		k=0;
		while (len(self.openList) != 0 and (self.openList[0].lt(self._calculateKey2(self.s_start))) or (self._getRHS(self.s_start) != self._getG(self.s_start))) :
			
			if (k > self.maxSteps) :
				print("At maxsteps");
				k += 1
				return -1;
			k += 1
			print "##### WHILE ##### ", k

			test = (self._getRHS(self.s_start) != self._getG(self.s_start));

			#lazy remove
			while(True) :
				#for lol in self.openList:
					#print "VARIAVEL LOL", lol.x, lol.y, lol.k.first(), lol.k.second()
				#print "len of openList", len(self.openList)
				if (len(self.openList) == 0):
					#print "return in 1"
					return 1;
				u = self.openList[0]
				del self.openList[0]
				#print "while", u.x, u.y
				if (not self._isValid(u)):
					continue;
				if (not (u.lt(self.s_start)) and (not test)):
					return 2;
				break;
			

			del self.openHash[u];

			k_old = clsState.State.fromState(u);

			if (k_old.lt(self._calculateKey(u))):  #u is out of date
				#print "###### IN LT ######"
				self._insert(u);
				#print "###### end IN LT ######"
			else :
				if (self._getG(u) > self._getRHS(u)):  #needs update (got better)
					#print "###### G>RHS ######"
					self._setG(u, self._getRHS(u));
					s = self._getPred(u);
					for i in s: 
						self._updateVertex(i);
					#print "###### end G>RHS ######"
				
				else: 						 # g <= rhs, state has got worse
					#print "###### senao ######"
					self._setG(u, float("inf"));
					s = self._getPred(u);

					for i in s :
						self._updateVertex(i);
					
					self._updateVertex(u);
					#print "###### end senao ######"
			
		 #while
		return 0;
	

	'''
	 * Returns a list of successor states for state u, since this is an
	 * 8-way graph this list contains all of a cells neighbours. Unless
	 * the cell is self._occupied, in which case it has no successors.
	'''
	def _getSucc( self, u):
		#print "getSucc", u.x, u.y
	
		s = [];
		#tempState;

		if (self._occupied(u)):
			return s;

		#Generate the successors, starting at the immediate right,
		#Moving in a clockwise manner
		tempState = clsState.State(u.x + 1, u.y, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);
		tempState = clsState.State(u.x + 1, u.y + 1, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);
		tempState = clsState.State(u.x, u.y + 1, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);
		tempState = clsState.State(u.x - 1, u.y + 1, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);
		tempState = clsState.State(u.x - 1, u.y, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);
		tempState = clsState.State(u.x - 1, u.y - 1, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);
		tempState = clsState.State(u.x, u.y - 1, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);
		tempState = clsState.State(u.x + 1, u.y - 1, clsPair.Pair(-1.0,-1.0));
		s.insert(0, tempState);

		return s;
	

	'''
	 * Returns a list of all the predecessor states for state u. Since
	 * this is for an 8-way connected graph, the list contains all the
	 * neighbours for state u. self._occupied neighbours are not added to the list
	'''
	def _getPred( self, u):
		#print "getPred", u.x, u.y
	
		s = [];

		tempState = clsState.State(u.x + 1, u.y, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
			s.insert(0, tempState);
		tempState = clsState.State(u.x + 1, u.y + 1, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
		 	s.insert(0, tempState);
		tempState = clsState.State(u.x, u.y + 1, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
		 	s.insert(0, tempState);
		tempState = clsState.State(u.x - 1, u.y + 1, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
		 	s.insert(0, tempState);
		tempState = clsState.State(u.x - 1, u.y, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
		 	s.insert(0, tempState);
		tempState = clsState.State(u.x - 1, u.y - 1, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
		 	s.insert(0, tempState);
		tempState = clsState.State(u.x, u.y - 1, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
		 	s.insert(0, tempState);
		tempState = clsState.State(u.x + 1, u.y - 1, clsPair.Pair(-1.0,-1.0));
		if (not self._occupied(tempState)):
		 	s.insert(0, tempState);

		return s;
	


	'''
	 * Update the position of the agent/robot.
	 * This does not force a replan.
	'''
	def updateStart( self, x,  y):
		#print "updateStart", x, y
	
		self.s_start.x = x;
		self.s_start.y = y;

		self.k_m += self._heuristic(self.s_last,self.s_start);

		self.s_start = self._calculateKey(self.s_start);
		self.s_last = self.s_start;

	

	'''
	 * This is somewhat of a hack, to change the position of the goal we
	 * first save all of the non-empty nodes on the map, clear the map, move the
	 * goal and add re-add all of the non-empty cells. Since most of these cells
	 * are not between the start and goal this does not seem to hurt performance
	 * too much. Also, it frees up a good deal of memory we are probably not
	 * going to use.
	'''
	def updateGoal( self, x,  y):
		#print "updateGoal", x, y
	
		toAdd = [];
		#tempPoint;

		for k, v in self.cellHash.iteritems():
			if (not self._close(v.cost, self.C1)) :
				tempPoint = clsPair.Pair( ipoint2( k.x, k.y), v.cost);
				toAdd.append(tempPoint);

		'''for entry in self.cellHash.entrySet() :
			if (not self._close(entry.getValue().cost, self.C1)) :
				tempPoint = clsPair.Pair(ipoint2(entry.getKey().x,entry.getKey().y), entry.getValue().cost);
				toAdd.append(tempPoint);'''
			
		

		self.cellHash = {};
		self.openHash = {};

		while(len(self.openList) != 0):
			del self.openList[0];

		self.k_m = 0;

		self.s_goal.x = x;
		self.s_goal.y = y;

		tmp = CellInfo();
		tmp.g = tmp.rhs = 0;
		tmp.cost = self.C1;

		self.cellHash[self.s_goal] = tmp;

		tmp = CellInfo();
		tmp.g = tmp.rhs = self._heuristic(self.s_start, self.s_goal);
		tmp.cost = self.C1;
		self.cellHash[self.s_start] = tmp;
		self.s_start = self._calculateKey(self.s_start);

		self.s_last = self.s_start;

		for tempPoint in toAdd:
			self.updateCell(tempPoint.first().x, tempPoint.first().y, tempPoint.second());

		'''iterator = toAdd.iterator();
		while(iterator.hasNext()) :
			tempPoint = iterator.next();
			updateCell(tempPoint.first().x, tempPoint.first().y, tempPoint.second());'''
		


	

	'''
	 * As per [S. Koenig, 2002]
	'''
	def _updateVertex( self, u):
		#print "updateVertex", u.x, u.y
	
		s = [];

		if (u.neq(self.s_goal)) :
			s = self._getSucc(u);
			tmp = float("inf");
			#tmp2;

			for i in s :
				tmp2 = self._getG(i) + self._cost(u,i);
				if (tmp2 < tmp):
					tmp = tmp2;
			
			if (not self._close(self._getRHS(u),tmp)):
				self._setRHS(u,tmp);
		

		if (not self._close(self._getG(u),self._getRHS(u))):
			self._insert(u);
		#print "end updateVertex", u.x, u.y
	

	'''
	 * Returns true if state u is on the open list or not by checking if
	 * it is in the hash table.
	'''
	def _isValid( self, u):
		#print "isValid", u.x, u.y
	

		for k, v in self.openHash.iteritems():
			if k.x == u.x and k.y == u.y:
				if (not self._close(self._keyHashCode(k),v)):
					return False;
				return True;
		return False
		
		'''if (openHash.get(u) == null) return false;
		if (!close(keyHashCode(u),openHash.get(u))) return false;
			return true;'''

	'''
	 * Sets the G value for state u
	'''
	def _setG( self, u,  g):
		#print "setG", u.x, u.y, g
	
		self.makeNewCell(u);
		for k, v in self.cellHash.iteritems():
			if k.x == u.x and k.y == u.y:
				self.cellHash[k].g = g
		'''self.cellHash.get(u).g = g;'''
	

	'''
	 * Sets the rhs value for state u
	'''
	def _setRHS( self, u,  rhs):
		#print "setRHS", u.x, u.y, rhs
	
		self.makeNewCell(u);

		for k, v in self.cellHash.iteritems():
			if k.x == u.x and k.y == u.y:
				v.rhs = rhs;

		'''self.cellHash.get(u).rhs = rhs;'''
	

	'''
	 * Checks if a cell is in the hash table, if not it adds it in.
	'''
	def makeNewCell( self, u):
		#print "makeNewCell", u.x, u.y
	
		for k, v in self.cellHash.iteritems():
			if k.x == u.x and k.y == u.y:
				return;
		tmp = CellInfo();
		tmp.g = tmp.rhs = self._heuristic(u,self.s_goal);
		tmp.cost = self.C1;
		self.cellHash[u] = tmp;
	

	'''
	 * updateCell as per [S. Koenig, 2002]
	'''
	def updateCell( self, x, y, val):
		#print "updateCell", x, y, val
	
		u = clsState.State.fromNone();
		u.x = x;
		u.y = y;

		if ((u.eq(self.s_start)) or (u.eq(self.s_goal))):
			return 1;

		self.makeNewCell(u);

		for k, v in self.cellHash.iteritems():
			if k.x == u.x and k.y == u.y:
				v.cost  = val;


		'''cellHash.get(u).cost = val;'''
		self._updateVertex(u);	
		#print("########## END UPDATE ##########");

	'''
	 * self._inserts state u into openList and openHash
	'''
	def _insert( self, u):
		#print "insert", u.x, u.y
	
		#iterator cur
		#csum;

		u = self._calculateKey(u);
		#cur = self.openHash.find(u);
		csum = self._keyHashCode(u);

		# return if cell is already in list. TODO: this should be
		# uncommented except it introduces a bug, I suspect that there is a
		# bug somewhere else and having duplicates in the openList queue
		# hides the problem...
		#if ((cur != self.openHash.end()) and (self._close(csum,cur->second))) return;

		self.openHash[u] = csum;

		'''integer = 0
		if len(self.openList) != 0:
			for integer in range(len(self.openList)):
				print integer
				if u.lt(self.openList[integer]):
					break
		self.openList.insert(integer, u)'''

		self.openList.append(u);
		self.openList.sort(key=lambda a: a.k.first())
		'''
		for lol in self.openList:
			print "VARIAVEL LOLA", lol.x, lol.y, lol.k.first(), lol.k.second()
		'''

	'''
	 * Returns the key hash code for the state u, this is used to compare
	 * a state that has been updated
	'''
	def _keyHashCode( self, u):
		#print "keyHashCode", u.x, u.y
	
		return (float)(u.k.first() + 1193*u.k.second())
	

	'''
	 * Returns true if the cell is occupied (non-traversable), False
	 * otherwise. Non-traversable are marked with a cost < 0
	'''
	def _occupied( self, u):
		#print "occupied", u.x, u.y
	
		#if the cellHash does not contain the State u
		for k, v in self.cellHash.iteritems():
			if k.x == u.x and k.y == u.y:
				return (v.cost < 0);
		return False;


		'''if (cellHash.get(u) == null)
			return false;
		return (cellHash.get(u).cost < 0);'''

	

	'''
	 * Euclidean cost between state a and state b
	'''
	def _trueDist( self, a, b):
		#print "trueDist", a.x, a.y, b.x, b.y
		x = a.x-b.x;
		y = a.y-b.y;
		return math.sqrt(x*x + y*y);
	

	'''
	 * Returns the cost of moving from state a to state b. This could be
	 * either the cost of moving off state a or onto state b, we went with the
	 * former. This is also the 8-way cost.
	'''
	def _cost( self, a, b):
		#print "cost", a.x, a.y, b.x, b.y
	
		xd = math.fabs(a.x-b.x);
		yd = math.fabs(a.y-b.y);
		scale = 1;

		if (xd+yd > 1):
			scale = self.M_SQRT2;

		for k, v in self.cellHash.iteritems():
			if k.x == a.x and k.y == a.y:
				return scale*v.cost;

		return scale*self.C1; 

	

	'''
	 * Returns true if x and y are within 10E-5, False otherwise
	'''
	def _close( self, x, y):
		#print "close", x, y
	
		if (x == float("inf") and y == float("inf")):
			return True;
		return (math.fabs(x-y) < 0.00001);
	

	def getPath(self):
		#print "getPath"
		return self.path;
	


	

def main():

	pf = DStarLite();
	pf.init(0,49,99,49);
	#pf.updateCell(0, 1, -1);
	pf.updateCell(2, 1, -1);
	pf.updateCell(2, 0, -1);
	pf.updateCell(2, 2, -1);
	pf.updateCell(3, 0, -1);''''''
	for x in xrange(1,99):
		pf.updateCell(49, x, -1);''''''

	print("Start node: (0,1)");
	print("End node: (3,1)");

	#Time the replanning
	#begin = System.currentTimeMillis();
	pf.replan();
	'''pf.updateGoal(3, 2);
	pf.updateStart(3, 3);
	pf.replan();'''
	#end = System.currentTimeMillis();

	#print("Time: " + (end-begin) + "ms");

	path = pf.getPath();
	cont = 0
	for i in path:
		print"x: ", i.x, " y: ", i.y, " cont: ", cont;
		cont = cont + 1;
	pf.init(45,48,99,49);

	pf.replan();

	path = pf.getPath();
	cont = 0
	for i in path:
		print"x: ", i.x, " y: ", i.y, " cont: ", cont;
		cont = cont + 1;

	pf.init(90,49,99,49);

	pf.replan();
	
	path = pf.getPath();
	cont = 0
	for i in path:
		print"x: ", i.x, " y: ", i.y, " cont: ", cont;
		cont = cont + 1;
	



if __name__ == "__main__":
	main()

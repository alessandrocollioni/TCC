import java.util.*;
/**
 *
 * @author daniel beard
 * http://danielbeard.wordpress.com
 * http://github.com/paintstripper
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
 * @compile javac DStarLite.java -Xlint
 * @run java DStarLite 
 *
 */
public class DStarLite implements java.io.Serializable{

	//Private Member variables
	private List<State> path = new ArrayList<State>();
	private double C1;
	private double k_m;
	private State s_start = new State();
	private State s_goal  = new State();
	private State s_last  = new State();
	private int maxSteps;
	private PriorityQueue<State>		openList = new PriorityQueue<State>();
	//Change back to private****
	public HashMap<State, CellInfo>	cellHash = new HashMap<State, CellInfo>();
	private HashMap<State, Float>		openHash = new HashMap<State, Float>();

	//Constants
	private double M_SQRT2 = Math.sqrt(2.0);

	//Default constructor
	public DStarLite()
	{
		maxSteps	= 8000000;
		C1			= 1;
	}

	//Calculate Keys
	public void CalculateKeys()
	{
			
	}

	/*
	 * Initialise Method
	 * @params start and goal coordinates
	 */
	public void init(int sX, int sY, int gX, int gY)
	{
		System.out.println("init " + sX +" "+ sY +" "+ gX +" "+ gY);
		cellHash.clear();
		path.clear();
		openHash.clear();
		while(!openList.isEmpty()) openList.poll();

		k_m = 0;

		s_start.x = sX;
		s_start.y = sY;
		s_goal.x  = gX;
		s_goal.y  = gY;

		CellInfo tmp = new CellInfo();
		tmp.g   = 0;
		tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start,s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;

	}

	/*
	 * CalculateKey(state u)
	 * As per [S. Koenig, 2002]
	 */
	private State calculateKey(State u)
	{
		System.out.println("calculateKey " + u.x +" "+ u.y);
		double val = Math.min(getRHS(u), getG(u));

		u.k.setFirst (val + heuristic(u,s_start) + k_m);
		u.k.setSecond(val);
		//System.out.println("end calculateKey " + u.x +" "+ u.y);
		return u;
	}

	/*
	 * Returns the rhs value for state u.
	 */
	private double getRHS(State u)
	{
		System.out.println("getRHS " + u.x +" "+ u.y);
		double aux = -20;
		if (u == s_goal){
			//System.out.println("return value RHS "+ 0);
			return 0;	
		} 

		//if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null){
			aux = heuristic(u, s_goal);
			//System.out.println("return aux RHS "+ aux);
			return aux;
		}
		//System.out.println("return rhs RHS "+ cellHash.get(u).rhs);
		return cellHash.get(u).rhs;
	}

	/*
	 * Returns the g value for the state u.
	 */
	private double getG(State u)
	{
		System.out.println("getG " + u.x +" "+ u.y);
		double aux = -20;
		//if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null){
			aux = heuristic(u,s_goal);
			//System.out.println("return aux G "+ aux);
			return aux;
		}
		//System.out.println("return G VALOR "+ cellHash.get(u).g);
		return cellHash.get(u).g;
	}

	/*
	 * Pretty self explanatory, the heuristic we use is the 8-way distance
	 * scaled by a constant C1 (should be set to <= min cost)
	 */
	private double heuristic(State a, State b)
	{
		System.out.println("heuristic " + a.x +" "+ a.y +" "+ b.x +" "+ b.y);
		return eightCondist(a,b)*C1;
	}

	/*
	 * Returns the 8-way distance between state a and state b
	 */
	private double eightCondist(State a, State b)
	{
		System.out.println("eightCondist " + a.x +" "+ a.y +" "+ b.x +" "+ b.y);
		double temp;
		double min = Math.abs(a.x - b.x);
		double max = Math.abs(a.y - b.y);
		if (min > max)
		{
			temp = min;
			min = max;
			max = temp;
		}
		//System.out.println("eightCondist RETURN " + ((M_SQRT2-1.0)*min + max));
		
		return ((M_SQRT2-1.0)*min + max);

	}

	public boolean replan()
	{
		System.out.println("replan ");
		path.clear();

		int res = computeShortestPath();
		//System.out.println("end from computeShortestPath");
		if (res < 0)
		{
			System.out.println("No Path to Goal");
			return false;
		}

		LinkedList<State> n = new LinkedList<State>();
		State cur = s_start;

		if (getG(s_start) == Double.POSITIVE_INFINITY)
		{
			System.out.println("No Path to Goal");
			return false;
		}

		while (cur.neq(s_goal))
		{
			path.add(cur);
			n = new LinkedList<State>();
			n = getSucc(cur);

			if (n.isEmpty())
			{
				System.out.println("No Path to Goal");
				return false;
			}

			double cmin = Double.POSITIVE_INFINITY;
			double tmin = 0;   
			State smin = new State();

			for (State i : n)
			{
				double val  = cost(cur, i);
				double val2 = trueDist(i,s_goal) + trueDist(s_start, i);
				val += getG(i);

				if (close(val,cmin)) {
					if (tmin > val2) {
						tmin = val2;
						cmin = val;
						smin = i;
					}
				} else if (val < cmin) {
					tmin = val2;
					cmin = val;
					smin = i;
				}
			}
			n.clear();
			cur = new State(smin);
			//cur = smin;
		}
		path.add(s_goal);
		return true;
	}

	/*
	 * As per [S. Koenig,2002] except for two main modifications:
	 * 1. We stop planning after a number of steps, 'maxsteps' we do this
	 *    because this algorithm can plan forever if the start is surrounded  by obstacles
	 * 2. We lazily remove states from the open list so we never have to iterate through it.
	 */
	private int computeShortestPath()
	{
		System.out.println("computeShortestPath ");
		LinkedList<State> s = new LinkedList<State>();

		if (openList.isEmpty()) return 1;

		int k=0;
		while ((!openList.isEmpty()) &&
			   (openList.peek().lt(s_start = calculateKey(s_start))) ||
			   (getRHS(s_start) != getG(s_start))) {

			if (k++ > maxSteps) {
				System.out.println("At maxsteps");
				return -1;
			}

			/*System.out.println("##### WHILE ##### "+ k);
			for (State lol : openList) {
				System.out.println("VARIAVEL LOL "+ lol.x+" "+ lol.y+" "+ lol.k.first()+" "+ lol.k.second() );
			}*/

			State u;

			boolean test = (getRHS(s_start) != getG(s_start));

			//lazy remove
			while(true) {
				
				//System.out.println("len of openList "+ openList.size());
				if (openList.isEmpty()){
					//System.out.println("return in 1");
					return 1;
				}
				u = openList.poll();
				//System.out.println("while "+ u.x+" "+ u.y);
				if (!isValid(u)) continue;
				if (!(u.lt(s_start)) && (!test)) return 2;
				break;
			}

			openHash.remove(u);

			State k_old = new State(u);

			if (k_old.lt(calculateKey(u))) { //u is out of date
				//System.out.println("###### IN LT ######");
				insert(u);
				//System.out.println("###### end IN LT ######");
			} else if (getG(u) > getRHS(u)) { //needs update (got better)
				//System.out.println("###### G>RHS ######");
				setG(u,getRHS(u));
				s = getPred(u);
				for (State i : s) {
					updateVertex(i);
				}
				//System.out.println("###### end G>RHS ######");
			} else {						 // g <= rhs, state has got worse
				//System.out.println("###### senao ######");
				setG(u, Double.POSITIVE_INFINITY);
				s = getPred(u);

				for (State i : s) {
					updateVertex(i);
				}
				updateVertex(u);
				//System.out.println("###### end senao ######");
			}
			//System.out.println("##### END WHILE ##### "+ k);
		} //while
		return 0;
	}

	/*
	 * Returns a list of successor states for state u, since this is an
	 * 8-way graph this list contains all of a cells neighbours. Unless
	 * the cell is occupied, in which case it has no successors.
	 */
	private LinkedList<State> getSucc(State u)
	{
		System.out.println("getSucc " + u.x +" "+ u.y);
		LinkedList<State> s = new LinkedList<State>();
		State tempState;

		if (occupied(u)) return s;

		//Generate the successors, starting at the immediate right,
		//Moving in a clockwise manner
		tempState = new State(u.x + 1, u.y, new Pair(-1.0,-1.0));
		s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y + 1, new Pair(-1.0,-1.0));
		s.addFirst(tempState);
		tempState = new State(u.x, u.y + 1, new Pair(-1.0,-1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y + 1, new Pair(-1.0,-1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y, new Pair(-1.0,-1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y - 1, new Pair(-1.0,-1.0));
		s.addFirst(tempState);
		tempState = new State(u.x, u.y - 1, new Pair(-1.0,-1.0));
		s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y - 1, new Pair(-1.0,-1.0));
		s.addFirst(tempState);

		return s;
	}

	/*
	 * Returns a list of all the predecessor states for state u. Since
	 * this is for an 8-way connected graph, the list contains all the
	 * neighbours for state u. Occupied neighbours are not added to the list
	 */
	private LinkedList<State> getPred(State u)
	{
		System.out.println("getPred " + u.x +" "+ u.y);
		LinkedList<State> s = new LinkedList<State>();
		State tempState;

		tempState = new State(u.x + 1, u.y, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y + 1, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);
		tempState = new State(u.x, u.y + 1, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y + 1, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y - 1, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);
		tempState = new State(u.x, u.y - 1, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y - 1, new Pair(-1.0,-1.0));
		if (!occupied(tempState)) s.addFirst(tempState);

		return s;
	}


	/*
	 * Update the position of the agent/robot.
	 * This does not force a replan.
	 */
	public void updateStart(int x, int y)
	{
		System.out.println("updateStart " + x +" "+ y);
		s_start.x = x;
		s_start.y = y;

		k_m += heuristic(s_last,s_start);

		s_start = calculateKey(s_start);
		s_last = s_start;

	}

	/*
	 * This is somewhat of a hack, to change the position of the goal we
	 * first save all of the non-empty nodes on the map, clear the map, move the
	 * goal and add re-add all of the non-empty cells. Since most of these cells
	 * are not between the start and goal this does not seem to hurt performance
	 * too much. Also, it frees up a good deal of memory we are probably not
	 * going to use.
	 */
	public void updateGoal(int x, int y)
	{
		System.out.println("updateGoal " + x +" "+ y);
		List<Pair<ipoint2, Double> > toAdd = new ArrayList<Pair<ipoint2, Double> >();
		Pair<ipoint2, Double> tempPoint;

		for (Map.Entry<State,CellInfo> entry : cellHash.entrySet()) {
			if (!close(entry.getValue().cost, C1)) {
				tempPoint = new Pair(
							new ipoint2(entry.getKey().x,entry.getKey().y),
							entry.getValue().cost);
				toAdd.add(tempPoint);
			}
		}

		cellHash.clear();
		openHash.clear();

		while(!openList.isEmpty())
			openList.poll();

		k_m = 0;

		s_goal.x = x;
		s_goal.y = y;

		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start, s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;

		Iterator<Pair<ipoint2,Double> > iterator = toAdd.iterator();
		while(iterator.hasNext()) {
			tempPoint = iterator.next();
			updateCell(tempPoint.first().x, tempPoint.first().y, tempPoint.second());
		}


	}

	/*
	 * As per [S. Koenig, 2002]
	 */
	private void updateVertex(State u)
	{
		System.out.println("updateVertex " + u.x +" "+ u.y);
		LinkedList<State> s = new LinkedList<State>();

		if (u.neq(s_goal)) {
			s = getSucc(u);
			double tmp = Double.POSITIVE_INFINITY;
			double tmp2;

			for (State i : s) {
				tmp2 = getG(i) + cost(u,i);
				if (tmp2 < tmp) tmp = tmp2;
			}
			if (!close(getRHS(u),tmp)) setRHS(u,tmp);
		}

		if (!close(getG(u),getRHS(u))) insert(u);
		//System.out.println("end updateVertex " + u.x +" "+ u.y);
	}

	/*
	 * Returns true if state u is on the open list or not by checking if
	 * it is in the hash table.
	 */
	private boolean isValid(State u)
	{
		System.out.println("isValid " + u.x +" "+ u.y);
		if (openHash.get(u) == null){
			//System.out.println("isValid " + u.x +" "+ u.y+ " RETURN FALSE 2");
			return false;
		} 
		if (!close(keyHashCode(u),openHash.get(u))){
			//System.out.println("isValid " + u.x +" "+ u.y+ " RETURN FALSE 1");
			return false;
		} 
		//System.out.println("isValid " + u.x +" "+ u.y+ " RETURN TRUE");
		return true;
	}

	/*
	 * Sets the G value for state u
	 */
	private void setG(State u, double g)
	{
		System.out.println("setG " + u.x +" "+ u.y+" "+ g);
		makeNewCell(u);
		cellHash.get(u).g = g;
	}

	/*
	 * Sets the rhs value for state u
	 */
	private void setRHS(State u, double rhs)
	{
		System.out.println("setRHS " + u.x +" "+ u.y+" "+ rhs);
		makeNewCell(u);
		cellHash.get(u).rhs = rhs;
	}

	/*
	 * Checks if a cell is in the hash table, if not it adds it in.
	 */
	private void makeNewCell(State u)
	{
		System.out.println("makeNewCell " + u.x +" "+ u.y);
		if (cellHash.get(u) != null) return;
		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(u,s_goal);
		tmp.cost = C1;
		cellHash.put(u, tmp);
	}

	/*
	 * updateCell as per [S. Koenig, 2002]
	 */
	public void updateCell(int x, int y, double val)
	{
		System.out.println("updateCell " + x +" "+ y+" "+ val);
		State u = new State();
		u.x = x;
		u.y = y;

		if ((u.eq(s_start)) || (u.eq(s_goal))) return;

		makeNewCell(u);
		cellHash.get(u).cost = val;
		updateVertex(u);
		//System.out.println("########## END UPDATE CELL ##########");
	}

	/*
	 * Inserts state u into openList and openHash
	 */
	private void insert(State u)
	{
		System.out.println("insert " + u.x +" "+ u.y);
		//iterator cur
		float csum;

		u = calculateKey(u);
		//cur = openHash.find(u);
		csum = keyHashCode(u);

		// return if cell is already in list. TODO: this should be
		// uncommented except it introduces a bug, I suspect that there is a
		// bug somewhere else and having duplicates in the openList queue
		// hides the problem...
		//if ((cur != openHash.end()) && (close(csum,cur->second))) return;

		openHash.put(u, csum);
		/*System.out.println("with " + u.x + " " + u.y + " OPEN HASH VALUE is: " + csum);
		for (Map.Entry<State, Float> entry : openHash.entrySet()) {
		    State key = entry.getKey();
		    Float value = entry.getValue();
		    System.out.println("CHAVE VALOR: "+ key.x+ " "+ key.y+ " "+ value);
		    // ...
		}*/
		
		openList.add(u);

		/*for (State lol : openList) {
			System.out.println("VARIAVEL LOLA "+ lol.x+" "+ lol.y+" "+ lol.k.first()+" "+ lol.k.second() );
		}*/
		
	}

	/*
	 * Returns the key hash code for the state u, this is used to compare
	 * a state that has been updated
	 */
	private float keyHashCode(State u)
	{
		System.out.println("keyHashCode " + u.x +" "+ u.y);
		return (float)(u.k.first() + 1193*u.k.second());
	}

	/*
	 * Returns true if the cell is occupied (non-traversable), false
	 * otherwise. Non-traversable are marked with a cost < 0
	 */
	private boolean occupied(State u)
	{
		System.out.println("occupied " + u.x +" "+ u.y);
		//if the cellHash does not contain the State u
		if (cellHash.get(u) == null)
			return false;
		return (cellHash.get(u).cost < 0);
	}

	/*
	 * Euclidean cost between state a and state b
	 */
	private double trueDist(State a, State b)
	{
		System.out.println("trueDist " + a.x +" "+ a.y +" "+ b.x +" "+ b.y);
		float x = a.x-b.x;
		float y = a.y-b.y;
		return Math.sqrt(x*x + y*y);
	}

	/*
	 * Returns the cost of moving from state a to state b. This could be
	 * either the cost of moving off state a or onto state b, we went with the
	 * former. This is also the 8-way cost.
	 */
	private double cost(State a, State b)
	{
		System.out.println("cost " + a.x +" "+ a.y +" "+ b.x +" "+ b.y);
		int xd = Math.abs(a.x-b.x);
		int yd = Math.abs(a.y-b.y);
		double scale = 1;

		if (xd+yd > 1) scale = M_SQRT2;

		if (cellHash.containsKey(a)==false) return scale*C1; 
		return scale*cellHash.get(a).cost;
	}

	/*
	 * Returns true if x and y are within 10E-5, false otherwise
	 */
	private boolean close(double x, double y)
	{
		System.out.println("close " + x +" "+ y );
		if (x == Double.POSITIVE_INFINITY && y == Double.POSITIVE_INFINITY){
			//System.out.println("close " + x +" "+ y +" RETURN TRUE");
			return true;
		} 
		//System.out.println("close " + x +" "+ y +" RETURN " + (Math.abs(x-y) < 0.00001));
		return (Math.abs(x-y) < 0.00001);
	}

	public List<State> getPath()
	{

		System.out.println("getPath");
		return path;
	}


	public static void main(String[] args)
	{
		DStarLite pf = new DStarLite();
		//pf.init(0,401,800,401);
		pf.init(1,1,49,49);
		pf.replan();
		List<State> path = pf.getPath();
		pf.updateStart(path.get(10).x, path.get(10).y);
		
		/*for (int i = -1; i < 801; i++) {
			pf.updateCell(800, i, -1);
			pf.updateCell(0, i, -1);
			pf.updateCell(i, 0, -1);
			pf.updateCell(i, 800, -1);
		}*/
		/*pf.updateCell(2, 1, -1);
		pf.updateCell(2, 0, -1);
		pf.updateCell(2, 2, -1);
		pf.updateCell(3, 0, -1);*/



		/*for (int i = 800; i > 100; i--) {
			pf.updateCell(400, i, -1);
			pf.updateCell(600, i, -1);
		}*/
		for (int i = 0; i < 40; i++) {
			pf.updateCell(25, i, -1);
		}


		System.out.println("Start node: (0,1)");
		System.out.println("End node: (3,1)");

		//Time the replanning
		long begin = System.currentTimeMillis();
		pf.replan();
		/*pf.updateGoal(3, 2);
		pf.updateStart(3, 3)
		pf.replan();*/
		long end = System.currentTimeMillis();

		//System.out.println("Time: " + (end-begin) + "ms");

		path = pf.getPath();
		for (State i : path)
		{
			System.out.println("x: " + i.x + " y: " + i.y);
		}

	}
}

class CellInfo implements java.io.Serializable
{
	public double g=0;
	public double rhs=0;
	public double cost=0;
}

class ipoint2
{
	public int x=0;
	public int y=0;

	//default constructor
	public ipoint2()
	{

	}

	//overloaded constructor
	public ipoint2(int x, int y)
	{
		this.x = x;
		this.y = y;
	}
}

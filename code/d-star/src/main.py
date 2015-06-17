import d_star as Dstar
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
	matrixMap[0][3] = 1
	matrixMap[0][4] = 1
	matrixMap[0][5] = 1
	matrixMap[0][6] = 1
	matrixMap[0][7] = 1
	matrixMap[0][8] = 1
	
	graph = Dstar.Graph(matrixMap, 10,10)

	planner = Dstar.Planner(graph, (0,0), (0,0))

	planner.replan()

	print(planner.path())
	planner.goal((0,1))
	planner.replan()
	print(planner.path())



if __name__ == "__main__" :
	main()


'''
def Main()

	g(sstart 2 ) := 0;

	parent(sstart) := sstart 3 ;

	open := [];

	open.Insert(sstart, g(sstart) + h(sstart 5 ));

	closed := [];

	while len(open) != 0  do

		s := open.Pop();

		if s = sgoal 9 then

			return "path found";

		closed := closed u {s};

		 

		[UpdateBounds(s)];

		foreach s'' in nghbrsvis(s) do

		 	if not(s'' in closed)
UpdateVertex
		 		if not(s'' in open)

		 			g(s'') := "inf";

		 			parent(s'') := NULL;

				UpdateVertex(s, s' 19 );

	return "no path found";


def UpdateVertex(s,s')

	if g(s) + c(s, s'') < g(s'')
		g(s'') := g(s) + c(s, s'');

		parent(s'') = s

		if s'' in open
			open.remove(s'')

		open.insert(s'', g(s'') + h(s''))


'''

# Sample code from http://www.redblobgames.com/pathfinding/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

#class Graph:
#    def __init__(self, image):
#        self.image = image
#    
#    def neighbors(self, point):
#        lista = []
#        lista.append(Point(point.x,point.y))
#        if point.x <= 0 :
#            lista.append(Point(point.x,point.y+1))
#        if point.x <= 500 :UMEROS DE ESTADOS EXPANDIDOS A-Star: 0

#            lista.append(Point(point.x,point.y-1))
#        if point.y <= 0 :
#            lista.append(Point(point.x+1,point.y))
#        if point.y <= 500 :
#            lista.append(Point(point.x-1,point.y))
#        return lista

import collections



class Graph():
    def __init__(self, width, height):
        self.weights = {}
        self.width = width
        self.height = height
        self.walls = []
    
    def cost(self, a, b):
        return self.weights.get(b, 1)

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def setObstacles(self, img, height, width):
        x = 0
        y = 0
        while ( x < height ):
            self.walls.append([])
            while ( y < width ):
                if (img[ y, x] == 0):
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
        if self.walls[x][y] == True:
            return None
        return id
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
        return self.elements.popleft()


import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    countOpenList = 0

    
    while not frontier.empty():
        current = frontier.get()
        countOpenList = countOpenList + 1
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    print 'NUMEROS DE ESTADOS EXPANDIDOS dijkstra:', countOpenList
    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    #manhatam distance
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    countOpenList = 0

    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            break
        countOpenList = countOpenList + 1
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                #print 'current: (%d,%d)' % next
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
                

    print 'NUMEROS DE ESTADOS EXPANDIDOS A-Star:', countOpenList
    return came_from, cost_so_far;


import abc
import numpy as np
import queue
from queue import PriorityQueue

#######################################################################
#
# The base class representation of a Graph with all the interface
# methods
#
#######################################################################
class Graph(abc.ABC):

    def __init__(self, numVertices, directed=False):
        self.numVertices = numVertices
        self.directed = directed

    @abc.abstractmethod
    def add_edge(self, v1, v2, weight):
        pass

    @abc.abstractmethod
    def get_adjacent_vertices(self, v):
        pass

    @abc.abstractmethod
    def get_indegree(self, v):
        pass

    @abc.abstractmethod
    def get_edge_weight(self, v1, v2):
        pass

    @abc.abstractmethod
    def display(self):
        pass


#######################################################################
#
# Represents a graph as an adjacency matrix. A cell in the matrix has
# a value when there exists an edge between the vertex represented by
# the row and column numbers.
# Weighted graphs can hold values > 1 in the matrix cells
# A value of 0 in the cell indicates that there is no edge
#
#######################################################################
class AdjacencyMatrixGraph(Graph):

    def __init__(self, numVertices, directed=False):
        super(AdjacencyMatrixGraph, self).__init__(numVertices, directed)

        self.matrix = np.zeros((numVertices, numVertices))


    def add_edge(self, v1, v2, weight=1):
        if v1 >= self.numVertices or v2 >= self.numVertices or v1 < 0 or v2 < 0:
            raise ValueError("Vertices %d and %d are out of bounds" % (v1, v2))

        if weight < 1:
            raise ValueError("An edge cannot have weight < 1")

        self.matrix[v1][v2] = weight
        if self.directed == False:
            self.matrix[v2][v1] = weight

    def get_adjacent_vertices(self, v):
        if v < 0 or v >= self.numVertices:
            raise ValueError("Cannot access vertex %d" % v)

        adjacent_vertices = []
        for i in range(self.numVertices):
            if self.matrix[v][i] > 0:
                adjacent_vertices.append(i)

        return adjacent_vertices

    def get_indegree(self, v):
        if v < 0 or v >= self.numVertices:
            raise ValueError("Cannot access vertex %d" % v)

        indegree = 0
        for i in range(self.numVertices):
            if self.matrix[i][v] > 0:
                indegree = indegree + 1

        return indegree

    def get_edge_weight(self, v1, v2):
        return self.matrix[v1][v2]

    def display(self):
        for i in range(self.numVertices):
            for v in self.get_adjacent_vertices(i):
                print(i, "-->", v)


#### Your code here #######
def UCS(graph, start, goal):
	#Declare a priority queue
	q = queue.PriorityQueue()
	#place the start value and cost = 0 in the queue
	q.put((0, start))
	#route = []
	#route.append(0)
	visited = []
	while not q.empty():
		#remove highest priority element
		cost, path_node = q.get()

		#If node not yet explored
		if path_node not in visited:
			visited.append(path_node)
			print("Currently visiting node: ", path_node)

			#If goal state is reached
			if path_node == goal:
				print("Total Cost is: ", cost)
				print("Visited Nodes List: ", visited)
				return visited

			#Get all the neighbors
			neighbors = graph.get_adjacent_vertices(path_node)
			#For every neighbor not yet visited
			for n in neighbors:
				if n not in visited:
					#Calculate the running cost i.e the cumulative cost
					#Add the cost and the node to the queue
					run_cost = cost + graph.get_edge_weight(path_node, n)
					#route.append(path_node)
					q.put((run_cost, n))


if __name__ == "__main__":
    g = AdjacencyMatrixGraph(10,True)
    g.add_edge(0, 1, 1)
    g.add_edge(0, 2, 1)
    g.add_edge(1, 3, 3)
    g.add_edge(2, 5, 2)
    g.add_edge(3, 6, 4)
    g.add_edge(3, 5, 2)
    g.add_edge(4, 6, 1)
    g.add_edge(4, 7, 5)
    g.add_edge(5, 4, 4)
    g.add_edge(6, 7, 1)
    g.add_edge(5, 0, 3)
    g.add_edge(5, 8, 1)
    g.add_edge(8, 4, 1)
    g.add_edge(8, 9, 3)
    g.add_edge(9, 7, 1)
    g.add_edge(7, 0, 30)

    for i in range(9):
         print("Adjacent to: ", i, g.get_adjacent_vertices(i))

    for i in range(9):
     print("Indegree: ", i, g.get_indegree(i))

    for i in range(9):
         for j in g.get_adjacent_vertices(i):
                print("Edge weight: ", i, " ", j, " weight: ", g.get_edge_weight(i, j))

    g.display()
UCS(g, 1, 8)


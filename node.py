import numpy as np
import math

class Node:
    # class for defining nodes in the abstract vector map

    def __init__(self, explored = False, los = True, vector = np.array([0,0])):
        # set bools for easy checking of whether node has been physically
        # explored and if there is *potentially* line of sight to the target
        self.explored = explored
        self.los = los

        # prepare variable for storing overall vector
        self.vector = vector
        self.score = 0

        # set lists for storing the local map as well as the connected nodes
        self.scan = []
        self.connections = []

    def hasLoS(self):
        return self.los

    def isExplored(self):
        return self.explored

    def setExplored(self):
        self.explored = True

    def get_vector(self):
        return self.vector
    
    def get_score(self):
        return self.score

    def add_connection(self, node, vector):
        # function to add connected nodes
        connection = Connection(node, vector)
        self.connections.append(connection)

    def get_connections(self):
        return self.connections

    def get_previous(self):
        return self.connections[0].get_node()

    def evaluate(self, target):
        self.score = np.linalg.norm(self.vector - target.vector)

    def is_target(self, target):
        return (math.isclose(self.vector[0], target.vector[0], abs_tol= 1e-3) and math.isclose(self.vector[1], target.vector[1], abs_tol= 1e-3))


class Connection:

    def __init__(self, node, vector):
        self.node = node
        self.vector = vector

    def get_node(self):
        # get the connected node
        return self.node

    def get_vector(self):
        # get vector between nodes
        return self.vector

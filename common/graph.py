from typing import Optional
from wpimath.geometry import Translation2d
import math
import heapq


class Node:
    position : Translation2d
    name : str

    def __init__(self, theName : str, thePosition : Translation2d):
        self.name = theName
        self.position = thePosition

class PriorityQueue:
    def __init__(self):
        self.elements: list[tuple[float, Node]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def push(self, item: Node, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def pop(self) -> Node:
        return heapq.heappop(self.elements)[1]

class Graph:

    def __init__(self):
        self.nodes: list[Node] = []
        self.namedNodes: dict[str, Node] = {}
        self.edges: dict[Node, list[Node]] = {}

    def write(self, filename: str):
        with open(filename, 'w') as file:
            for node in self.nodes:
                file.write(f"{node.name} {node.position.x} {node.position.y}\n")
            file.write("---\n")
            for node in self.nodes:
                for neighbor in self.neighbors(node):
                    file.write(f"{node.name} {neighbor.name}\n")

    def nodeCount(self) -> int:
        return len(self.nodes)
    
    def addNode(self, node: Node):
        self.nodes.append(node)
        assert node.name not in self.namedNodes
        self.namedNodes[node.name] = node

    def getNode(self, name : str) -> Node | None:
        return self.namedNodes[name]

    def __addEdgeUnidirectional(self, nameFrom: str, nameTo: str):
        nodeFrom = self.namedNodes[nameFrom]
        nodeTo = self.namedNodes[nameTo]
        assert nodeFrom is not None
        assert nodeTo is not None
        if nodeFrom not in self.edges:
            self.edges[nodeFrom] = []
        assert nodeTo not in self.edges[nodeFrom]
        self.edges[nodeFrom].append(nodeTo)

    def addEdgeBidirectional(self, nameFrom: str, nameTo: str):
        self.__addEdgeUnidirectional(nameFrom, nameTo)
        self.__addEdgeUnidirectional(nameTo, nameFrom)

    def neighbors(self, node: Node) -> list[Node]:
        if self.edges.get(node) is None:
            return []
        return self.edges[node]
    
    def findClosest(self, position: Translation2d) -> Node | None:
        closestNode = None
        closestDistance = math.inf
        for node in self.nodes:
            distance = node.position.distance(position)
            if closestNode is None or distance < closestDistance:
                closestNode = node
                closestDistance = distance
        return closestNode

class AStar:
    @staticmethod
    def heuristic(a: Node, b: Node) -> float:
        return a.position.distance(b.position) # TODO distance carrée?

    @staticmethod
    def generatePath(graph : Graph, start: Node, goal: Node) -> list[Node]:
        queue = PriorityQueue()
        queue.push(start, 0)
        came_from: dict[Node, Optional[Node]] = {}
        cost_so_far: dict[Node, float] = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not queue.empty():
            current: Node = queue.pop()           
            if current == goal: # On est arrivé à destination
                break
            
            for next in graph.neighbors(current):
                new_cost = cost_so_far[current] + current.position.distance(next.position)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + AStar.heuristic(next, goal)
                    queue.push(next, priority)
                    came_from[next] = current
        return AStar.__reconstruct_path(came_from, start, goal)
    
    @staticmethod
    def __reconstruct_path( came_from: dict[Node, Optional[Node]],
                            start: Node, goal: Node) -> list[Node]:
        current: Node = goal
        path: list[Node] = []
        if goal not in came_from: # no path was found
            return []
        while current != start:
            path.append(current)
            optionnalCurrent = came_from[current]
            if optionnalCurrent is None:
                return [] # ?? Impossible
            else:
                current = optionnalCurrent

        path.append(start) # optional
        path.reverse() # optional
        return path
    
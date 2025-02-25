import heapq
import numpy as np


class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g  # Cost from start to current node
        self.h = h  # Heuristic cost from current node to goal
        self.f = g + h  # Total cost

    def __lt__(self, other):
        return self.f < other.f

class AStar3D:
    def __init__(self, start, goal, grid_size, obstacles, bounds, diagonal_flag=True):
        self.start = start
        self.goal = goal
        self.grid_size = grid_size
        self.obstacles = obstacles
        self.bounds = bounds
        self.diagonal = diagonal_flag

        self.path = None

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def get_neighbors(self, node):
        x, y, z = node.position
        neighbors = []
        # If diagonal moves should be included:
        if self.diagonal: 
            moves = [
                (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                (0, 0, 1), (0, 0, -1),
                (1, 1, 0), (-1, -1, 0), (1, -1, 0), (-1, 1, 0),
                (1, 0, 1), (-1, 0, -1), (1, 0, -1), (-1, 0, 1),
                (0, 1, 1), (0, -1, -1), (0, 1, -1), (0, -1, 1),
                (1, 1, 1), (-1, -1, -1), (1, -1, -1), (-1, 1, -1),
                (1, 1, -1), (-1, -1, 1), (1, -1, 1), (-1, 1, 1)
            ]
        else:
            moves = [
            (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1)
            ]
        for dx, dy, dz in moves:
            new_pos = (x + dx * self.grid_size, y + dy * self.grid_size, z + dz * self.grid_size)
            if self.bounds[0] <= new_pos[0] <= self.bounds[1] and self.bounds[2] <= new_pos[1] <= self.bounds[3] and self.bounds[4] <= new_pos[2] <= self.bounds[5]:
                if not any(obs[0] <= new_pos[0] <= obs[0] + obs[3] and
                           obs[1] <= new_pos[1] <= obs[1] + obs[4] and
                           obs[2] <= new_pos[2] <= obs[2] + obs[5] for obs in self.obstacles):
                    neighbors.append(new_pos)
        return neighbors

    def find_path(self):
        open_list = []
        closed_set = set()
        start_node = Node(self.start, None, 0, self.heuristic(self.start, self.goal))
        heapq.heappush(open_list, start_node)
        
        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position == self.goal:
                self.path = []
                while current_node:
                    self.path.append(current_node.position)
                    current_node = current_node.parent
                return self.path[::-1]
            
            closed_set.add(current_node.position)
            
            for neighbor_pos in self.get_neighbors(current_node):
                if neighbor_pos in closed_set:
                    continue
                neighbor_node = Node(neighbor_pos, current_node, current_node.g + self.grid_size, self.heuristic(neighbor_pos, self.goal))
                
                if any(open_node.position == neighbor_node.position and open_node.f <= neighbor_node.f for open_node in open_list):
                    continue
                
                heapq.heappush(open_list, neighbor_node)
        
        return None  # No path found
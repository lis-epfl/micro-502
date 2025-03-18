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
        self.inflation = 0.25

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
                if not any(obs[0] - self.inflation <= new_pos[0] <= obs[0] + obs[3] + self.inflation and
                           obs[1] - self.inflation <= new_pos[1] <= obs[1] + obs[4] + self.inflation and
                           obs[2] - self.inflation <= new_pos[2] <= obs[2] + obs[5] + self.inflation and
                           self.line_intersects_aabb(node.position, new_pos, obs) for obs in self.obstacles):
                    neighbors.append(new_pos)
        return neighbors

    def line_intersects_aabb(self, start, end, box):
        """Check if a line segment intersects an axis-aligned bounding box (AABB)."""
        box_min = np.array([box[0] - self.inflation, box[1] - self.inflation, box[2] - self.inflation])
        box_max = np.array([box[0] + box[3] + self.inflation, box[1] + box[4] + self.inflation, box[2] + box[5] + self.inflation])

        start = np.array(start)
        end = np.array(end)

        tmin, tmax = 0, 1
        direction = end - start

        for i in range(3):  # Check x, y, z axes
            if abs(direction[i]) < 1e-6:  # Parallel to slab
                if start[i] < box_min[i] or start[i] > box_max[i]:
                    return False
            else:
                t1 = (box_min[i] - start[i]) / direction[i]
                t2 = (box_max[i] - start[i]) / direction[i]
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
                if tmin > tmax:
                    return False
        return True

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
                self.path = self.path[::-1]
                self.path = self.remove_unnecessary_points(self.path)
                return self.path
            
            closed_set.add(current_node.position)
            
            for neighbor_pos in self.get_neighbors(current_node):
                if neighbor_pos in closed_set:
                    continue
                neighbor_node = Node(neighbor_pos, current_node, current_node.g + self.grid_size, self.heuristic(neighbor_pos, self.goal))
                
                if any(open_node.position == neighbor_node.position and open_node.f <= neighbor_node.f for open_node in open_list):
                    continue
                
                heapq.heappush(open_list, neighbor_node)
        
        return None  # No path found

    def remove_unnecessary_points(self, path):
        if not path:
            return path

        optimized_path = [path[0]]
        for i in range(2, len(path)):
            for obs in self.obstacles:
                if self.line_intersects_aabb(optimized_path[-1], path[i], obs):
                    optimized_path.append(path[i - 1])
        optimized_path.append(path[-1])
        return optimized_path

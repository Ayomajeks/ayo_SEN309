# ==========================
# Drone Delivery Navigation
# ==========================

import heapq
from collections import deque

# --------- Graph Definition ---------
graph = {
    'A': {'B': 2, 'C': 5, 'D': 1},
    'B': {'A': 2, 'D': 2, 'E': 3},
    'C': {'A': 5, 'D': 2, 'F': 3},
    'D': {'A': 1, 'B': 2, 'C': 2, 'E': 1, 'F': 4},
    'E': {'B': 3, 'D': 1, 'G': 2},
    'F': {'C': 3, 'D': 4, 'G': 1},
    'G': {'E': 2, 'F': 1, 'H': 3},
    'H': {'G': 3}
}

heuristic = {
    'A': 7,
    'B': 6,
    'C': 6,
    'D': 4,
    'E': 2,
    'F': 2,
    'G': 1,
    'H': 0
}

start = 'A'
goal = 'H'

# ===================================
# Depth-First Search (DFS)
# ===================================
def dfs(graph, start, goal):
    visited = set()
    path = []
    nodes_expanded = 0

    def dfs_helper(node):
        nonlocal nodes_expanded
        if node in visited:
            return False

        visited.add(node)
        path.append(node)
        nodes_expanded += 1

        if node == goal:
            return True

        for neigbour in graph[node]:
            if dfs_helper(neigbour):
                return True   

        path.pop() #For backtracking
        return False             
        

    dfs_helper(start)
    return path, len(path)-1, nodes_expanded

# ===================================
# Breadth-First Search (BFS)
# ===================================
def bfs(graph, start, goal):
    visited = set()
    queue = deque()
    queue.append((start, [start]))
    nodes_expanded = 0

    while queue:
        node, path = queue.popleft()
        nodes_expanded += 1

        if node == goal:
            return path, len(path)-1, nodes_expanded

        if node not in visited:
            visited.add(node)

            for neigbour in graph[node]:
                queue.append((neigbour, path + [neigbour]))    

    return None , 0, nodes_expanded           

# ===================================
# Uniform Cost Search (UCS)
# ===================================
def ucs(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start, [start]))
    explored = set()
    nodes_expanded = 0

    while frontier:
        cost, node, path = heapq.heappop(frontier)
        nodes_expanded += 1

        if node == goal:
            return path, cost, nodes_expanded

        if node not in explored:
            explored.add(node)

            for neigbour, weight in graph[node].items():
                heapq.heappush(frontier, (cost + weight, neigbour, path + [neigbour]))

    return None, 0, nodes_expanded         

# ===================================
# A* Search
# ===================================
def a_star(graph, start, goal, heuristic):
    frontier = []
    heapq.heappush(frontier, (heuristic[start], 0, start, [start]))
    explored = set()
    nodes_expanded = 0

    while frontier:
        f, cost, node, path = heapq.heappop(frontier)
        nodes_expanded += 1

        if node == goal:
            return path, cost, nodes_expanded

        if node not in explored:
            explored.add(node)

            for neigbour, weight in graph[node].items():
                new_cost = cost + weight
                new_f = new_cost + heuristic[neigbour]

                heapq.heappush(frontier, (new_f, new_cost, neigbour, path + [neigbour]))

    return None, 0, nodes_expanded
# ===================================
# Run and Compare
# ===================================
if __name__ == "__main__":
    dfs_path, dfs_steps, dfs_nodes = dfs(graph, start, goal)
    bfs_path, bfs_steps, bfs_nodes = bfs(graph, start, goal)
    ucs_path, ucs_cost, ucs_nodes = ucs(graph, start, goal)
    astar_path, astar_cost, astar_nodes = a_star(graph, start, goal, heuristic)

    print("DFS path:", dfs_path, "| Steps:", dfs_steps, "| Nodes expanded:", dfs_nodes)
    print("BFS path:", bfs_path, "| Steps:", bfs_steps, "| Nodes expanded:", bfs_nodes)
    print("UCS path:", ucs_path, "| Cost:", ucs_cost, "| Nodes expanded:", ucs_nodes)
    print("A* path:", astar_path, "| Cost:", astar_cost, "| Nodes expanded:", astar_nodes)
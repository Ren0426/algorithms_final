import heapq

# Algorithm 1: Dijkstra's Algorithm (Shortest Path)
def dijkstra(graph, start, end):
    # Initialization 
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    previous = {node: None for node in graph}
    queue = [(0, start)]

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_node == end:
            # Route recovery
            path = []
            while current_node:
                path.insert(0, current_node)
                current_node = previous[current_node]
            return path, distances[end]

        for neighbor, weight in graph.get(current_node, []):
            if weight < 0:
                return "Error: Negative weight edge detected"
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    return "Unreachable"

# Algorithm 2: Prim's Algorithm (Minimum Spanning Tree)
def prim(graph, start):
    visited = set()
    queue = [(0, start, None)]  # (weight, current_node, previous_node)
    mst = []
    total_cost = 0

    while queue:
        weight, current, prev = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        if prev is not None:
            mst.append((prev, current, weight))
            total_cost += weight
        for neighbor, edge_weight in graph.get(current, []):
            if neighbor not in visited:
                heapq.heappush(queue, (edge_weight, neighbor, current))

    return mst, total_cost

# Algorithm 3: Dynamic MST using Kruskal's Algorithm
def dynamic_mst(graph, start, remove_edges, add_edges):
    # Generate edge lists
    edges = []
    for node in graph:
        for neighbor, weight in graph[node]:
            if (node, neighbor) not in remove_edges and (neighbor, node) not in remove_edges:
                edges.append((weight, node, neighbor))

    for u, v, w in add_edges:
        edges.append((w, u, v))

    # UKruskal's union find
    parent = {}
    def find(u):
        if parent[u] != u:
            parent[u] = find(parent[u])
        return parent[u]
    def union(u, v):
        root_u = find(u)
        root_v = find(v)
        if root_u != root_v:
            parent[root_v] = root_u
            return True
        return False

    nodes = set()
    for _, u, v in edges:
        nodes.add(u)
        nodes.add(v)
    parent = {node: node for node in nodes}

    mst = []
    total_cost = 0
    for weight, u, v in sorted(edges):
        if union(u, v):
            mst.append((u, v, weight))
            total_cost += weight
        if len(mst) == len(nodes) - 1:
            break

    if len(mst) != len(nodes) - 1:
        return "It is impossible to connect all nodes"
    return mst, total_cost

# Test Cases 
example_graph = {
    "A": [("B", 4), ("C", 2)],
    "B": [("A", 4), ("C", 1), ("D", 5)],
    "C": [("A", 2), ("B", 1), ("D", 8), ("E", 10)],
    "D": [("B", 5), ("C", 8), ("E", 2)],
    "E": [("C", 10), ("D", 2)]
}

print("--- Algorithm 1: Dijkstra ---")
print(dijkstra(example_graph, "A", "E"))  # Expected path A-C-B-D-E, Cost: 11
print(dijkstra(example_graph, "A", "B"))  # Expected path A-C-B, Cost: 3
print(dijkstra(example_graph, "A", "Z"))  # Expected: Unreachable

# Add negative to the graph
graph_with_negative = {
    "A": [("B", -1)],
    "B": [("A", -1)]
}
print(dijkstra(graph_with_negative, "A", "B"))  # Expected: Error message

print("\n--- Algorithm 2: Prim ---")
print(prim(example_graph, "A"))  # Expected MST and Total Cost

example_graph_with_isolated = example_graph.copy()
example_graph_with_isolated["F"] = []
print(prim(example_graph_with_isolated, "A"))  # "F" should not appear in MST

print("\n--- Algorithm 3: Dynamic MST ---")
print(dynamic_mst(example_graph, "A", [("C", "E")], [("B", "E", 3)]))  # MST with modified edge
print(dynamic_mst(example_graph, "A", [("A", "C")], []))  # A-C removed
print(dynamic_mst(example_graph, "A", [
    ("A", "B"), ("A", "C"), ("B", "C"), ("B", "D"), ("C", "D"), ("D", "E"), ("C", "E")
], []))  # All edges removed

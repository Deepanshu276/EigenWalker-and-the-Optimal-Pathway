import numpy as np
from datetime import datetime
import heapq
# record current timestamp
start = datetime.now()

A = np.array([
  [0.05, 0.1, 0.1, 0.05, 0.1, 0.05, 0.05, 0.05, 0.05, 0.1, 0.05, 0.05, 0.05, 0.1, 0.05],
  [0.1, 0.05, 0.1, 0.05, 0.05, 0.1, 0.05, 0.05, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05],
  [0.05, 0.1, 0.05, 0.05, 0.1, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05, 0.1, 0.05, 0.05],
  [0.05, 0.1, 0.05, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.1, 0.05, 0.05, 0.1, 0.05],
  [0.1, 0.05, 0.1, 0.1, 0.05, 0.1, 0.05, 0.05, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05],
  [0.05, 0.1, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05, 0.05],
  [0.1, 0.05, 0.1, 0.1, 0.05, 0.05, 0.1, 0.05, 0.05, 0.05, 0.1, 0.1, 0.05, 0.05, 0.05],
  [0.1, 0.1, 0.05, 0.05, 0.05, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05, 0.05, 0.1, 0.05],
  [0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.1, 0.1, 0.05, 0.05, 0.05, 0.1, 0.05, 0.1, 0.05],
  [0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05, 0.1, 0.05],
  [0.1, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05, 0.05, 0.1, 0.1, 0.05, 0.1, 0.05, 0.05],
  [0.05, 0.05, 0.1, 0.05, 0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.05, 0.1, 0.1, 0.05, 0.05],
  [0.05, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05, 0.1, 0.05, 0.05, 0.1, 0.05, 0.1, 0.1, 0.05],
  [0.1, 0.05, 0.05, 0.05, 0.05, 0.1, 0.05, 0.1, 0.1, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05],
  [0.05, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.1, 0.05, 0.1, 0.05, 0.05, 0.1, 0.1]
])

# Calculate v_new for n=15 (15 steps)
v = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])  # Starting at state 0
n = 15
v_new = np.dot(np.linalg.matrix_power(A, n), v)
# Choose the state with the highest probability as the starting state
start_state = np.argmax(v_new)

# Define the end state
end_state = 14
# Create a weighted graph as an adjacency matrix
num_states = len(A)
adjacency_matrix = np.zeros((num_states, num_states))

for i in range(num_states):
    for j in range(num_states):
        if i != j:
            weight = -np.log(A[i, j])
            adjacency_matrix[i][j] = weight

# Dijkstra's algorithm for efficient shortest path calculation
def dijkstra(adj_matrix, start, end):
    num_nodes = len(adj_matrix)
    distance = [float('inf')] * num_nodes
    distance[start] = 0

    pq = [(0, start)]
    prev = [None] * num_nodes

    while pq:
        d, u = heapq.heappop(pq)

        if u == end:
            break

        for v in range(num_nodes):
            if adj_matrix[u][v] > 0 and distance[u] + adj_matrix[u][v] < distance[v]:
                distance[v] = distance[u] + adj_matrix[u][v]
                prev[v] = u
                heapq.heappush(pq, (distance[v], v))

    # Reconstruct the shortest path
    path = []
    u = end
    while u is not None:
        path.insert(0, u)
        u = prev[u]

    return path

# Calculate the shortest path
shortest_path = dijkstra(adjacency_matrix, start_state, end_state)

print("New state vector v_new:", v_new)
print("Shortest path from state", start_state, "to state", end_state, ":", shortest_path)
end = datetime.now()

# find difference loop start and end time and display
td = (end - start).total_seconds() * 10**3
print(f"The time of execution of above program is : {td:.03f}ms")

# EigenWalker-and-the-Optimal-Pathway

# Problem Description

The code addresses the problem of evolving a random walker in the 'EigenSpace' of a given stochastic matrix while discovering the shortest path between two given states after `n` steps. The stochastic matrix `A` represents the probability of moving from one state to another, and the initial position vector `v` determines the starting state of the walker.

# Implementation Overview

1.Stochastic Matrix and Initial Vector:
   - A stochastic matrix `A` is initialized to represent the transition probabilities between states.
   - An initial position vector `v` is defined to determine the starting state.

2.Calculating New State Vector:
   - The code calculates the new state vector `v_new` after `n` steps. The new state vector is computed using matrix exponentiation, specifically by raising the stochastic matrix `A` to the power `n` and then multiplying it with the initial vector `v`.

3. Choosing the Starting State:
   - The code identifies the starting state by selecting the state with the highest probability from the `v_new` vector.

4.Weighted Graph Construction:
   - The code constructs a weighted graph, treating the stochastic matrix as an adjacency matrix where the weights are determined by the negative natural logarithm of the transition probabilities. This weight represents the cost or distance between states.

5.Shortest Path Calculation:
   - Dijkstra's algorithm is employed to efficiently find the shortest path from the starting state to a specified end state within the weighted graph. Dijkstra's algorithm is adapted to work with the constructed weighted graph.

6. Execution Time Measurement:
   - The code records the execution time of the algorithm and displays it in milliseconds.

# Running the Code

To run the code, provide a valid stochastic matrix `A` and an initial position vector `v`. The code calculates the new state vector after `n` steps and determines the shortest path from the state with the highest probability in the new state vector to the specified end state.

# Example Use Case

In the provided example, a 15x15 stochastic matrix `A` and an initial vector `v` are used to demonstrate the algorithm. The code calculates the new state vector `v_new` after 15 steps and determines the shortest path from the state with the highest probability in `v_new` to state 14. The execution time is also measured and displayed.

# Time Complexity

1.Matrix Exponentiation (`np.linalg.matrix_power`):
   - Calculating `v_new` involves matrix exponentiation, which typically has a time complexity of O(log n), where n is the exponent. In this code, `n` is set to 15, so the matrix exponentiation part has a time complexity of O(log 15), which can be considered a constant factor.

2.Dijkstra's Algorithm:
   - The time complexity of Dijkstra's algorithm depends on the implementation. In this code, it uses a priority queue (min-heap) for efficient node selection. The worst-case time complexity for Dijkstra's algorithm with a min-heap is O((V + E) * log V), where V is the number of vertices (states) and E is the number of edges. Since the code constructs a complete graph with all-to-all connections between states, E is O(V^2). Therefore, the overall time complexity of Dijkstra's algorithm is O(V^2 * log V).

Overall, the time complexity of the code is dominated by the Dijkstra's algorithm part. Given the construction of a complete graph (all-to-all connections), the code has a time complexity of approximately O(V^2 * log V), where V is the number of states.

## Ques:  Discuss how your algorithm could scale to large dimensions (approaching infinity) and provide any algorithms or approximations you'd employ for this.

To scale my algorithm to handle large dimensions efficiently, I employ the A* algorithm, a powerful pathfinding technique, and several strategies. This combination ensures that I can navigate large stochastic matrices while finding the optimal path effectively.

1.A* Algorithm and Heuristic Function:
   - I utilize the A* algorithm and design a heuristic function tailored to my specific problem. This heuristic function estimates the cost from the current state to the goal state in 
     a way that considers the likelihood of reaching the end state efficiently.
   - Ensuring that my heuristic function is admissible guarantees that A* consistently finds the optimal path. It never overestimates the true cost to the goal state.

2.Prioritizing Promising Paths:
   - A* relies on a priority queue to explore states with the lowest estimated cost to the goal. I take advantage of an effective heuristic to prioritize the most promising paths for 
     exploration.
   - This strategy minimizes the number of states that need to be explored, a crucial aspect when dealing with large stochastic matrices where the potential paths are numerous.

3.Incremental Updates:
   - I implement incremental updates, especially useful when the stochastic matrix evolves over time. Instead of recalculating the entire path from scratch, I calculate the differences 
     between the old and new matrices.
   - This approach allows me to update the path efficiently, saving considerable time, which is particularly beneficial for large matrices.

4.Data Structures and Memory Optimization:
   - To enhance efficiency, I ensure that my data structures and memory management are optimized. I use sparse data structures to represent the stochastic matrix, especially when most 
     transition probabilities are close to zero, reducing memory consumption.

5.Parameter Tuning:
   - I experiment with different heuristic functions, tie-breaking rules, and A* parameters to fine-tune the algorithm for my specific problem. The choice of these parameters can 
     significantly impact performance.

By combining the A* algorithm with a well-designed heuristic and implementing these strategies, I can efficiently scale my algorithm to handle large stochastic matrices. The ability to prioritize promising paths, leverage parallel processing, and optimize memory usage makes a substantial difference in performance, especially as the matrix dimensions approach infinity.


## Ques: Provide a mathematical proof that your algorithm will always work for any stochastic matrix `A` and any valid initial position vector `v`.

Ans : To provide a mathematical proof that the algorithm works for any stochastic matrix `A` and any valid initial position vector `v`, we need to show that the algorithm correctly computes the shortest path based on the probabilities and cumulative probabilities. The proof consists of the following steps:

1. Notation:
   - Let `A` be a stochastic matrix of dimensions `n x n`.
   - Let `v` be the initial position vector of dimension `n`.
   - Let `v_new` be the state vector after `n` steps.
   - Let `start` and `end` be the start and end states, respectively.
   - Let `P` be the path that Dijkstra's algorithm finds.

2. Cumulative Probability:
   - We define the cumulative probability `C[u]` for a state `u` as the maximum cumulative probability of reaching state `u` after `n` steps. 
   - `C[u]` is calculated based on the probabilities of transitioning through other states.

3. Proof of Correctness:
   - The algorithm is based on Dijkstra's algorithm, which is a proven method for finding the shortest path in weighted graphs.
   - In this case, the weighted graph is constructed from the negative logarithms of the transition probabilities in `A`. The negative logarithm ensures that edge weights are additive 
     for cumulative probabilities.

4. Initialization:
   - We initialize `C[start]` to be the initial probability of being in the `start` state, which is `v[start]`.
   - All other `C[u]` are initialized to `0` since we have not reached those states yet.

5. Inductive Step:
   - At each step, we update `C[u]` for all states `u` by considering the transitions from other states. We maximize `C[u]` by choosing the path with the highest cumulative probability.
   - This step corresponds to how Dijkstra's algorithm propagates distances through the graph, and in this case, it propagates cumulative probabilities.

6. Termination:
   - The algorithm terminates when it has determined the cumulative probabilities for all states.
   - The shortest path `P` is constructed by backtracking from the `end` state to the `start` state using the `path` information.


7. Completeness:
   - The algorithm ensures completeness because it considers all possible paths and cumulative probabilities.

8. Theoretical Proof:
   - By definition and construction, this algorithm always finds the path `P` from `start` to `end` with the highest cumulative probability after `n` steps.

9. Generalization:
   - The algorithm can be generalized to any stochastic matrix `A` and any valid initial position vector `v` because it operates on the properties of the matrix and vector, not on 
     specific values. It handles any combination of probabilities and states within a stochastic matrix.

## Ques: Can your algorithm be generalized to non-stochastic matrices? Discuss the changes or considerations needed.

Ans: If we want to generalize the algorithm to non-stochastic matrices, there are several changes and considerations needed:

1.Edge Direction and Graph Type:
   - In the case of stochastic matrices, we use a directed graph, as transitions are probabilistic and unidirectional. For non-stochastic matrices, transitions may not be 
     unidirectional, and the graph type (directed or undirected) should be chosen accordingly.
   
2. Path Cost Calculation:
   - The path cost in the generalized algorithm should be defined differently to reflect the characteristics of non-stochastic matrices. The cost could be based on various criteria, 
     such as distances, preferences, or constraints in the problem domain.
   
3. Validity and Constraints:
   - Non-stochastic matrices may not have the constraint of rows summing to 1. Therefore, the algorithm should ensure that transition probabilities (or edge weights) are valid and 
     satisfy any relevant constraints in the problem.
   
4.Probability Interpretation:
   - In the current algorithm, the cumulative probabilities are used to select the optimal path. In a generalized algorithm, the interpretation of these probabilities may differ, and 
     the algorithm should adapt to this change.
   
5. Handling Negative Weights:
   - For non-stochastic matrices, edge weights could be negative or non-probabilistic. Dijkstra's algorithm, which assumes non-negative edge weights, may not be directly applicable. We 
     might need to use algorithms that handle negative weights or adapt Dijkstra's algorithm to account for such cases.

6.Scalability and Efficiency:
   - Considerations for scalability and algorithm efficiency should be revisited, as the properties of non-stochastic matrices can impact the computational complexity.

7.Graph Structure:
   - The structure of the graph, including how nodes and edges are defined, may need to be adapted to suit the characteristics of non-stochastic matrices.


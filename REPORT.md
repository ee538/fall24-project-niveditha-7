# EE538 Final Project - Fall 2024 - TrojanMap

# AUTHOR DETAILS
| Name                  	| Email            	| USC ID     	|
|-----------------------	|------------------	|------------	|
| NIVEDITHA MADEGOWDA 	            | madegowd@usc.edu 	| 9517006149 	|
| LEONARDO ROBLES       	| lroblesa@usc.edu 	| 6431758800 	            |
2. Link youtube: [Youtube Video Link](https://www.youtube.com/watch?v=5uOlsDhjrDM&t=25s)

## Project Overview
The TrojanMap project is a comprehensive application designed to manage and analyze geographic data using graph-based approaches. It includes functionalities such as shortest path computation, traveling salesman problem solving, and data extraction using regular expressions. The project demonstrates practical applications of algorithms for solving real-world navigation and optimization problems, tailored for the USC area. The overview of different functionalities is as follows :

![Project Overview](ubuntu/images/Overview.JPG)

****************************************************************************************************
## Implemented Functionalities

### 1. Autocomplete

#### **Description**
The Autocomplete function retrieves all locations matching a given partial name (prefix). The function is case-insensitive, ensuring flexibility in user input.

#### **Implementation**
1. Converts the input prefix and all location names in the dataset to lowercase for case-insensitive matching.
2. Iterates through all nodes and checks if the location name starts with the given prefix using `substr`.
3. Returns all matching location names as a vector.

#### **Performance**
- **Time Complexity**: `O (n*m)`, where 'n' is the number of locations and 'm' is the average length of the location names.

![Autocomplete](ubuntu/images/autocomplete.JPG)

****************************************************************************************************
### **Item 2. Find Location**

#### **Description**
Retrieves latitude, longitude, or ID of a location.

#### **Implementation**
- **`GetLat` and `GetLon`**:
  - Check if the given location ID exists in the dataset.
  - If it exists, return the latitude or longitude; otherwise, return `-1`. 
- **`GetID`**:
  - Iterates through the dataset to find the ID of the location name.
  - Returns an empty string if the name does not exist.
#### **Performance**
- **Time Complexity**:  ` O (n) `, where 'n' is number of locations/nodes.
-  Time taken: 1 ms

![Find the Location](ubuntu/images/FindLocation.JPG)

****************************************************************************************************
### **Item 2-2. Edit Distance**

#### **Description**
Calculates the minimum number of operations (insertions, deletions, substitutions) required to convert one string into another. Used in `FindClosestName`.

#### **Implementation**
1. Converts both input strings to lowercase for consistency.
2. Uses dynamic programming to populate a 2D matrix, where each cell represents the edit distance up to that point.
3. Returns the value at the bottom-right corner of the matrix as the result.
#### **Example**
- **Input**: `"Univercity"`
- **Output**: `"University of Southern California"`

#### **Performance**
- **Time Complexity**: ` O (m * n) `, where 'm' and 'n' are the lengths of the two strings.



![ Edit Distance](ubuntu/images/EditDistance.JPG)

****************************************************************************************************
### **Item 3. Get All Categories**

#### **Description**
Returns all unique location categories from the dataset.

#### **Implementation**
1. Extracts attributes (categories) from all nodes and inserts them into a `std::set` to ensure uniqueness.
2. Converts the set into a vector for the output.

#### **Example**
- **Output**: `["restaurant", "bank", "school", ...]`

#### **Performance**
- **Time Complexity**: ` O (m * n) `, where 'm' is the average number of attributes per location and 'n' the number of locations

---

![ Get All Categories](ubuntu/images/GetAllCategories.JPG)

****************************************************************************************************
### **Item 4. Get All Locations from a Category**

#### **Description**
The `GetAllLocationsFromCategory` function retrieves all locations belonging to a specific category. It filters the dataset to match the provided category and returns the corresponding location IDs.

#### **Implementation**
- Iterates through all nodes in the dataset.
- Checks if each node's `category` matches the given input.
- Adds the node IDs of matching locations to a results vector.
- #### **Performance**
- **Time Complexity**: ` O (n) `, where 'n' is the number of locations in the dataset.

---

![ Get All Locations From Category](ubuntu/images/GetAllLocationsFromCategory.JPG)

****************************************************************************************************
### **Item 5. Regular Expression Matching**

#### **Description**
The `GetLocationRegex` function filters location names based on a user-provided regular expression pattern. It enables users to dynamically query location names using flexible matching rules.

#### **Implementation**
1. Accepts a `std::regex` object as input, representing the pattern to match.
2. Iterates through all nodes in the dataset.
3. For each node:
   - Uses `std::regex_match` to check if the node's name matches the provided regex pattern.
4. Adds all matching location IDs to a results vector.
5. Returns the vector containing IDs of matching locations.
#### **Performance**
- **Time Complexity**: ` O (n * l) `, where:
  - 'n' is the number of locations.
  - 'l' is the average length of the location names.

#### **Error Handling**
- Invalid regular expressions throw a `std::regex_error`.
- The implementation catches this error and provides meaningful feedback to the user.

---
![Regular Expression Matching](ubuntu/images/regex.jpg)

****************************************************************************************************
### Item 6. **Shortest Path**

#### **Description**
The `CalculateShortestPath` function computes the shortest path between two locations in the TrojanMap dataset using two algorithms:
1. **Dijkstra’s Algorithm**: Efficient for non-negative edge weights.
2. **Bellman-Ford Algorithm**: Handles graphs with negative edge weights but is slower compared to Dijkstra's.

#### **Implementation**

1. **Dijkstra's Algorithm**:
   - **Initialization**:
     - Retrieve the unique IDs of the start and destination locations.
     - Initialize a priority queue to explore nodes based on the shortest known distance.
     - Set up a `distances` map to store the shortest distance from the start node to each node, initialized to infinity, and set the distance of the start node to `0`.
   - **Processing**:
     - While the priority queue is not empty:
       - Extract the node with the shortest distance.
       - For each neighbor of the current node:
         - Calculate the potential new distance.
         - If the new distance is shorter than the stored distance, update it and push the neighbor into the queue.
   - **Path Reconstruction**:
     - Use a `predecessors` map to backtrack from the destination to the start location, constructing the shortest path.
   - **Time Complexity**: ` O ((V + E) log V)) `, where 'V' is the number of vertices and 'E' is the number of edges.

2. **Bellman-Ford Algorithm**:
   - **Initialization**:
     - Retrieve the unique IDs of the start and destination locations.
     - Initialize a `short_distance_from_start` map with distances set to infinity, except for the start node, which is set to `0`.
   - **Relaxation**:
     - For (V-1) iterations (where 'V' is the number of vertices
       - For each edge in the graph, calculate the distance from the current node to its neighbors.
       - If a shorter distance is found, update the distance and store the predecessor.
       - If no update occurs in an iteration, terminate early.
   - **Path Reconstruction**:
     - Backtrack from the destination to the start using the predecessor map to construct the path.
   - **Time Complexity**: ` O (V * E)`, where 'V' is the number of vertices and 'E' is the number of edges.


#### **Performance**
- **Dijkstra's Complexity**: `O ((n + e) log n))`  
- **Bellman-Ford Complexity**: `O (n * e)`
- where 'n' is the number of vertices and 'e' is the number of edges.


![Shortest path](ubuntu/images/Shortestpath.JPG)

** runtime of the algorithm for several examples.**

| Point A to Point B      | Dijkstra | Bellman Ford optimmized| Bellman Ford non-optimized|
| -------------------- | ----------- |-------|-----|
|                      |  t1 (ms)    | t2 (ms)    |   t3 (ms) |
|Ralphs to Target      |  56         | 5313  | 1506141           |
|Chevron to Target     |  86         | 9060  | 1502278     |
|Chase - Cheebos Burger|  163        | 6500  |  1525787    |
|George Lucas Instructional Building to Arco                     | 94            |  7273     | 1513648     |
|Trojan Grounds (Starbucks) to  Ralphs                     |  75           |7141       |      |
|Chipotle to DASH                      |58             |7181       |      |
|Hope Manor to The Pearl      |  58           | 12639      |      |
|Dulce to the Barber Shop                     | 103           |  5682     |      |
|Vermont & Exposition 1  to Distributing Station 31|   116         | 8302      |      |
|Stimson House to Hoover & 28th |  63           |  8426     |      |
|Felix Chevrolet to Pico & Grand|  156           |  9728     |      |
| Hill & Adams to Driveway    |    72         |    10411   |      |
| Lyons Center to Venice & Flower| 124           |  10796     |      |
|Olive & 12th to  Lululemon |       92      |  10600     |      |
|Grand & 39th to Chick-fil-A | 35            |  9269     |      |
| The Coffee Bean & Tea Leaf to Birnkrant |     45        | 8485      |      |
|                      |             |       |      |
|                      |             |       |      |


****************************************************************************************************
### Comparison of Dijkstra and Bellman Ford
#### **Comparison**
- **Dijkstra’s Algorithm**:
  - Faster for graphs with non-negative edge weights.
  - Suitable for most real-world navigation problems.
- **Bellman-Ford Algorithm**:
  - Handles graphs with negative edge weights.
  - Useful for detecting negative weight cycles but is computationally more expensive.

#### **Error Handling**
- Ensures the start and end locations are valid nodes in the graph.
- For Bellman-Ford, checks for the presence of negative weight cycles and handles them gracefully.

### Additional Notes:
- **Practical Application**: Used for navigation tasks, such as finding the quickest route between two locations.
- **Optimization**: Dijkstra’s implementation leverages a min-heap for efficient priority queue operations.
- **Use Case**:  
  - Input locations can be selected by name or ID. The output path is displayed as a vector of location names, along with the total distance.

The following image shows the time taken of every function according to the destination.

![Comparison of Dijkstra and Bellman Ford](ubuntu/images/DijkstravsBellmanFord.JPG)

****************************************************************************************************
### **Item 7. Cycle Detection**

#### **Description**
The `CycleDetection` function determines whether there is a cycle in a subgraph defined by specific geographical bounds (latitude and longitude).

#### **Implementation**

1. **Subgraph Construction**:
   - The function takes as input a square boundary defined by four coordinates: `left`, `right`, `upper`, and `lower`.
   - It identifies all nodes that fall within this boundary using the `GetSubgraph` helper function.
   - Builds an adjacency list (`subgraph_map`) for the subgraph by including only edges where both nodes are inside the square.

2. **Cycle Detection with DFS**:
   - Uses a recursive helper function `DFS_CycleDetection` to traverse the graph.
   - During traversal:
     - Marks nodes as visited.
     - Recursively visits each neighbor.
     - Detects a cycle if a neighbor has already been visited and is not the immediate parent node.

3. **Edge Cases**:
   - If the subgraph has fewer than three nodes, no cycle can exist.
   - Handles disconnected nodes by iterating over all unvisited nodes in the subgraph.

3. **Result**:
   - If a cycle is found during DFS, the function returns `true`.
   - If the DFS completes without finding a cycle, the function returns `false`.
--> Time taken: 16 ms

#### **Performance**
- **Time Complexity**: ` O (V + E)`, where:
  - 'V': Number of nodes in the subgraph.
  - 'E': Number of edges in the subgraph.
- **Space Complexity**: ` O (V) `, for storing visited nodes and recursive call stack.

 #### **Error Handling**
- Returns `false` if the subgraph has fewer than three nodes, as a cycle is impossible.
- Ensures only valid edges (within the square boundary) are included in the subgraph.

![ Cycle detection](ubuntu/images/cycle.png)

****************************************************************************************************

### **Item 8. Topological Sort**

#### **Description**
The `Topological Sort` functionality determines the order in which tasks or locations can be processed based on dependencies. It is designed to work with a directed acyclic graph (DAG) represented as a set of nodes (locations) and edges (dependencies).

#### **Implementation**

1. **Input Parsing**:
   - The function reads two input files:
     - A CSV file containing the list of location names.
     - A CSV file specifying the dependencies between these locations (directed edges).

2. **Graph Construction**:
   - Builds a directed graph using an adjacency list representation.
   - Creates a map of **in-degrees** to count the number of dependencies for each node:
     - Initializes in-degrees for all nodes to `0`.
     - Updates the in-degree count based on dependencies.

3. **Node Processing**:
   - Uses a queue to process nodes with an in-degree of `0` (no prerequisites).
   - For each processed node:
     - Adds the node to the result list.
     - Decreases the in-degree of its neighbors.
     - If a neighbor's in-degree becomes `0`, it is added to the queue.

4. **Cycle Detection**:
   - After processing all nodes:
     - If the result list contains all input nodes, a valid topological order exists.
     - Otherwise, the graph contains a cycle, and the function returns an empty list.

#### **Example**
- **Input**:  
  - Nodes: `[A, B, C, D, E]`  
  - Edges: `[A -> B, A -> C, B -> D, C -> D, D -> E]`
- **Output**:  
  - Topological Order: `[A, B, C, D, E]`

#### **Performance**
- **Time Complexity**:`  O (V + E)`, where:
  - 'V' is the number of vertices.
  - 'E' is the number of edges.
- The adjacency list representation ensures efficient traversal and updates.

#### **Error Handling**
- Handles cases where:
  - Input files are empty (returns an empty list).

#### **Examples**
- Given: 
      - location_names = {"Ralphs", "Chick-fil-A", "KFC"}
      - dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"Chick-fil-A", "KFC"}}
      - Here, {"Ralphs","KFC"} means that Tommy must go to Ralphs prior to KFC.

- **Output**:
      - For example, given an input a csv file with dependencies the expected output wouldbe the sorted locations for the given csv file
      1. Grand & 30th
      2. Grand & Adams
      3. University Park
      4. Jefferson/USC 

![Topological Sort](ubuntu/images/TopologicalSort.JPG)

****************************************************************************************************
### **Item 9. Traveling Trojan**

#### **Description**
The `TravelingTrojan` function solves the Traveling Salesperson Problem (TSP) for a given subset of locations on the TrojanMap. The goal is to find the shortest possible route that visits each location exactly once and returns to the starting point. The implementation explores multiple approaches, including **Brute Force**, **Backtracking**, and optimization techniques like **2-opt** and **3-opt**.

#### **Implementation**

1. **Brute Force**:
   - Generates all permutations of the locations.
   - Calculates the total distance for each permutation.
   - Tracks the permutation with the minimum total distance.
   - **Pros**: Guarantees the optimal solution.
   - **Cons**: Computationally expensive for large datasets due to factorial complexity.

2. **Backtracking**:
   - Explores paths recursively and calculates their distances.
   - Prunes paths that exceed the current best-known distance to save computation time.
   - **Pros**: More efficient than brute force due to pruning.
   - **Cons**: Still expensive for larger datasets.
     
3. **2-Opt Optimization**:
   - Starts with an initial path (for example: the order of input locations).
   - Iteratively swaps two edges to check if the total distance decreases.
   - Stops when no further improvement is possible.
   - **Pros**: Significantly faster than brute force for larger datasets.
   - **Cons**: May not guarantee the optimal solution.

4. **3-Opt Optimization**:
   - Extends 2-opt by swapping three edges at a time for potential improvement.
   - Provides better optimization compared to 2-opt at the cost of increased computation time.
   - **Pros**: More accurate than 2-opt.
   - **Cons**: Slower than 2-opt.

#### **Performance**

| **Method**       | **Time Complexity**     | **Accuracy**          |
|------------------|-------------------------|-----------------------|
| Brute Force      | O (n!)                 | Optimal (guaranteed)  |
| Backtracking     | O (n!) (pruned)        | Optimal (guaranteed)  |
| 2-Opt            | O (n²)                | Approximate           |
| 3-Opt            | O (n³)                | Better Approximation  |


#### **Error Handling**
- Validates that the input list of locations is non-empty.
- Ensures all input locations exist in the dataset.
- Handles edge cases where:
  - The subset contains only one or two locations (trivial solutions).
  - The graph is disconnected, preventing a complete tour.

It is worth noting how the time to run the function using Brute force w backtracking decreased 66.67 %
This lies in their approach to explore and solve problems. While both methods are exhaustive, backtracking 
introduces an optimization to systematically discard solutions that cannot possibly work, reducing unnecessary computation.

![Traveling Trojan](ubuntu/images/TravelingTrojan.JPG)


High level functions, in this case n³, tend to be slower to solve, this is one reason 2 opt is faster.
A good practice learnt here is to try to avoid high order polynomios/functions if possible. 
If interested in seeing an animation of how Brute force, 2 opt and 3 opt animation works please see the Youtube video
 [youtube ref](https://www.youtube.com/watch?v=5uOlsDhjrDM&t=25s)


- **Edge Cases**:
  - If the graph contains no edges, the function returns an empty path.
  - If a valid solution cannot be found (e.g., disconnected graph), appropriate error messages are returned.
Factorial growth of brute force `O (n!)` will eventually outpace the polynomial growth of 2-opt `O (n²)` and
3-opt `O (n³)` optimizations.
****************************************************************************************************
### **9b. Traveling Trojan (Benchmarks)**

#### **Description**
To evaluate the performance of various Traveling Salesperson Problem (TSP) implementations, we benchmarked the following algorithms:
1. **Brute Force**
2. **Backtracking**
3. **2-Opt**
4. **3-Opt**

Benchmarks were conducted using the Google Benchmark library for input sizes ranging from 2 to 10 locations.

#### **Implementation**
1. Benchmarked each algorithm by iterating over a range of input sizes using predefined sample locations.
2. Measured runtime performance (in milliseconds) for the following:
   - **Brute Force vs Backtracking**
   - **2-Opt vs 3-Opt**
3. Plotted the results to visualize the performance trends.

#### **Benchmark Results**
1. **Brute Force vs Backtracking**:
   - As shown in the graph below, Brute Force demonstrates exponential growth in runtime with increasing input size due to its `O(n!)` complexity.
   - Backtracking is significantly faster, leveraging pruning to reduce the number of explored paths.

   ![Brute Force vs Backtracking](ubuntu/images/BrutVsBack.png)

   - **Insights**:
     - Brute Force becomes infeasible for input sizes greater than 10.
     - Backtracking offers considerable improvements but is still impractical for large inputs.

2. **2-Opt vs 3-Opt**:
   - The graph below illustrates the trade-off between accuracy and runtime.
   - While 3-Opt produces better solutions, its runtime grows faster than 2-Opt due to increased complexity.

   ![2-Opt vs 3-Opt](ubuntu/images/2vs3.png)

   - **Insights**:
     - For smaller input sizes, both algorithms have similar runtimes.
     - For larger datasets, 3-Opt is slower, making 2-Opt a more practical choice for approximate solutions.
       
#### **Conclusion**
- Brute Force and Backtracking are suitable for small datasets but become computationally prohibitive as input size grows.
- For larger datasets, heuristic algorithms like 2-Opt and 3-Opt offer a practical trade-off between runtime and solution quality.

### **Additional Notes**
- Benchmarks were conducted using sample data and are representative of real-world scenarios.
- The visualizations provide a clear comparison of runtime growth trends, highlighting the scalability challenges of exact algorithms versus heuristic approaches.

****************************************************************************************************
### **Item 10. Find Nearby**

#### **Description**
The `FindNearby` function identifies all locations within a specified radius of a given location and filters them based on a specific category (e.g., "restaurant"). The results are then sorted by proximity.


#### **Implementation**

1. **Input Validation**:
   - Checks if the given location ID is valid.
   - Ensures the radius is non-negative.

2. **Filter by Category**:
   - Iterates through all locations in the dataset.
   - Filters nodes based on whether they belong to the specified category.

3. **Calculate Distance**: (had already been given to us)
   - For each filtered location, calculates the geographical distance from the given location using the Haversine formula:
  
4. **Filter by Radius**:
   - Only include locations within the specified radius.

5. **Sort by Distance**:
   - Sorts the filtered locations in ascending order of distance.

6. **Return Top Results**:
   - Returns the top 'k' results, where 'k' is given by the user.

#### **Performance**
- **Time Complexity**:
  - ` O (n)` for filtering by category.
  - ` O (n log n) ` for sorting the filtered results.
- Overall: ` O (n log n) `, where 'n' is the number of locations in the dataset.

#### **Error Handling**
- Returns an appropriate message if:
  - The input location is invalid.
  - No locations within the radius match the category.
  - The radius is set to zero or a negative value.

### **Additional Notes**
- **Optimization**:
  - The implementation leverages geographical filtering to avoid unnecessary distance calculations.
  - Sorting ensures the most relevant locations are presented first.

- **Edge Cases**:
  - If the dataset is empty, the function returns an empty list.
  - If no locations match the category, the function informs the user.

---

![Find Nearby](ubuntu/images/FindNearby.JPG)

****************************************************************************************************
### **Item 11. Trojan Path**

#### **Description**
The **Trojan Path** function calculates the shortest path that visits all given locations in a specified order and returns to the starting point. 

#### **Implementation**

1. **Precomputing Pairwise Shortest Paths**:
   - Before solving the TSP, the function uses `CalculateShortestPath_Dijkstra` to compute the shortest path between all pairs of input locations.
   - Stores the pairwise shortest paths and distances in `path_map` and `distance_map`, respectively.

2. **Backtracking Logic**:
   - A recursive helper function `backtrack` explores all possible paths:
     - **Base Case**:
       - If all locations have been visited, the algorithm computes the round-trip distance by adding the precomputed distance from the last location back to the starting point.
       - Updates the `min_distance` and `optimal_path` if a shorter path is found.
     - **Recursive Case**:
       - For each unvisited location:
         - Add the location to `current_path`.
         - Retrieve the distance to the next location from the precomputed `distance_map` and recursively call `backtrack`.
         - After the recursive call, remove the location from `current_path` (backtracking step).

3. **Pruning**:
   - Paths with a cumulative distance greater than the current `min_distance` are skipped to reduce unnecessary computations.

4. **Edge Case Handling**:
   - If the input list of locations is empty, the function returns an empty result.
   - For a single location, it returns the same location with a distance of `0`.
     
#### **Performance**
- **Dijkstra’s Preprocessing**:
  - **Time Complexity**: ` O (V * (E + V log V)) `, where:
    - 'V': Number of locations.
    - 'E': Number of edges in the graph.
      
- **Backtracking**:
  - **Time Complexity**: ` O (n!)`, where 'n' is the number of input locations.
  - Pruning reduces the effective search space, improving runtime compared to naive brute force.

- **Space Complexity**: ` O (n)`, due to the recursive stack and storage of the current path.

#### **Error Handling**
- Handles invalid inputs such as:
  - Empty list of locations.
  - Locations not present in the dataset.
- Returns appropriate messages if a valid path cannot be constructed.

### **Additional Notes**

- **Edge Cases**:
  - Single location: Returns the same location with a distance of `0`.
  - Two locations: Directly calculates the distance between them and returns.

![Trojan Path](ubuntu/images/TrojanPath.JPG)

****************************************************************************************************
## Item 12 - Check Path
### **Check Path**

#### **Description**
The `CheckPath` function determines whether a vehicle can travel from one location to another with a limited fuel tank capacity. It accounts for the need to refill at gas stations along the way and ensures the journey is feasible within the constraints.

#### **Implementation**

1. **Union-Find Approach**:
   - The function employs a **union-find** data structure to group locations that are reachable within the gas tank constraint.
   - Each node starts as its own parent in the union-find structure.

2. **Building Connected Components**:
   - Iterates over all locations and their neighbors.
   - If the distance between a location and its neighbor is less than or equal to the gas tank size, the two locations are united into the same connected component.

3. **Query Verification**:
   - After building the connected components, the function verifies if the start and destination locations belong to the same component.
   - If they share the same component, a path exists; otherwise, it does not.

4. **Interactive Path Query**:
   - Allows users to input:
     - Start location.
     - Destination.
     - Gas tank size.
   - Processes the input and provides feedback on whether the path exists under the given constraints.


#### **Performance**
- **Time Complexity**: ` O (V * E)`, where:
  - 'V' is the number of nodes (locations).
  - 'E' is the number of edges (connections between locations).
- Efficient exploration is achieved by limiting the search to nodes within the current fuel range.
- **Space Complexity**: ` O (V) `, for storing the union-find parent structure.

#### **Error Handling**
- Returns `"No path exists"` if:
  - The graph is disconnected, making the destination unreachable.
  - Fuel capacity is insufficient to reach any gas station from the start location.
- Handles edge cases where:
  - Start and destination are the same (trivial solution).
  - Input fuel capacity or locations are invalid.

### **Additional Notes**

- **Applications**:
  - Real-world navigation systems that account for fuel efficiency and availability of gas stations.
  - Autonomous vehicle route planning with battery or fuel constraints.

- **Optimization**:
  - Limits search space by precomputing reachable nodes within the maximum fuel range.
  - Dynamic programming can be used for larger graphs to track minimum fuel consumption for each node.

#### **Error Handling**
- Returns `false` for invalid input cases:
  - If either the start or destination location does not exist.
  - If the input locations are not in a valid format..

![Check Path](ubuntu/images/CheckPath.JPG)

****************************************************************************************************
## Item 13 - Exit
- To exit out of the menu
- 
## **Conclusion**

The TrojanMap project demonstrates the application of graph theory and advanced algorithms to solve practical problems involving geographical data. By implementing a variety of functionalities such as shortest path computation, traveling salesperson problem (TSP) solutions, cycle detection, and regex-based queries, the project bridges theoretical knowledge with real-world applications.

Through this project, we successfully:
- Developed an efficient graph-based navigation system for the USC area.
- Implemented optimized algorithms like Dijkstra’s and Bellman-Ford for shortest path computation.
- Solved complex optimization problems like TSP using brute force and heuristic techniques.
- Enhanced usability by adding features like autocomplete and category-based filtering.

This project not only provided technical insights but also highlighted the importance of designing scalable and user-friendly solutions for real-world problems.
We were able to learn a lot from C++ theory , such as: recursion, pointers, and memory management and data structures too like adjacency lists and priority 
queues to make graph operations more efficient and used regex for dynamic location-based queries. Collaborating with
Git version control throughout the process ensured smooth teamwork and effective project management, making the 
experience both technically enriching and rewarding. 

## **Lessons Learned**

1. **Algorithmic Efficiency**:
   - Understanding the trade-offs between accuracy and performance in algorithms like brute force vs. heuristic approaches for TSP.
   - Realizing the importance of data structures like priority queues and adjacency lists for efficient graph traversal.

2. **Practical Application of Graph Theory**:
   - Implementing graph-based algorithms reinforced our theoretical knowledge.
   - Learned to handle edge cases such as disconnected graphs and cyclic dependencies.

3. **Dynamic Programming and Optimization**:
   - Leveraged dynamic programming to optimize TSP solutions, reducing redundant computations.
   - Implemented optimization techniques like 2-opt and 3-opt to improve route calculations.

4. **Error Handling and Robustness**:
   - Gained experience in designing robust systems by accounting for edge cases and invalid inputs.
   - Improved user feedback mechanisms with meaningful error messages.

5. **Software Development Skills**:
   - Enhanced our ability to write clean, modular, and maintainable code.
   - Used version control effectively to collaborate and manage changes.

6. **Practical Use of C++ Features**:
   - Improved understanding of STL (Standard Template Library) for data handling and algorithm implementation.
   - Explored advanced C++ concepts like lambda functions, regex, and dynamic memory allocation.

7. **Team Collaboration**:
   - Strengthened teamwork skills through collaborative problem-solving and effective communication.
   - Leveraged tools like Git for version control and task management.

---








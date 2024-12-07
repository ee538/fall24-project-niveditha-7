# EE538 Final Project - Fall 2024 - TrojanMap

1. Authors : Niveditha Madegowda and Leonardo Robles
2. Link youtube: [Youtube Video Link](https://www.youtube.com/watch?v=5uOlsDhjrDM&t=25s)

## Project Overview
The project is divided into the following functions.
![Project Overview](ubuntu/images/Overview.JPG)

****************************************************************************************************
## Item 1 - Autocomplete
Suggests locations that match a partial input name.
--> For example, given the input: Uni
            the expected value: "all locations starting with 'Uni'.."
--> Time complexity: O (n)
--> Time taken: 2 ms

![Autocomplete](ubuntu/images/autocomplete.JPG)

****************************************************************************************************
## Item 2 - Find the Location
Suggests locations that match a partial input name.
*Gets the latitude and longitude by first retrieving the ID using GetID
and then looking up the coordinates*
--> For example, given the input: USC Credit Union
            the expected value: "latitude: 34.0257  longitude: -118.285"
--> Time complexity: O (n)
--> Time taken: 1 ms

![Find the Location](ubuntu/images/FindLocation.JPG)

****************************************************************************************************
## Item 2-B -  Edit Distance
*Computes the edit distance between two strings using dynamic 
programming, this way if the input does not correspond to a valid location it will suggest a valid location that matches the characters given*
--> For example, given the input: The Coffe a tea le
            the expected value: unmathced location
                            did u mean The Coffee Beam & Tea Leaf instead of The Coffe a tea le? [y/n] ? //if y
                             "latitude: 34.0172  longitude: -118.282"
--> Time complexity: O (mxn)
--> Time taken: 2 ms

![ Edit Distance](ubuntu/images/EditDistance.JPG)

****************************************************************************************************
## Item 3 -  Get All Categories
*Gets all the categorys*
--> For example, running this function should display every category
--> Time complexity: O (nxm)

![ Get All Categories](ubuntu/images/GetAllCategories.JPG)

****************************************************************************************************
## Item 4 -  Get All Locations From Category
*Finds the location of every element within a category*
--> For example, given the input: bank
            the expected value: {"9591449441", "9591449465","5237417651"}
--> Time complexity: O (n)

![ Get All Locations From Category](ubuntu/images/GetAllLocationsFromCategory.JPG)

****************************************************************************************************
## Item 5 - Get location Regex   
![]()

****************************************************************************************************
## Item 6 -  Shortest path 
*Finds the shortest path using Dijkstra and Bellman Ford algorithm*
///////////// Dijkstra //////////////////////////////////////////////
--> For example, given the start location: Ralphs
            and given the destination: University Park
            the expected value: 0.77816 miles
--> Time complexity: O ((n+m) log n)
--> Time taken: 26 ms

///////////// Bellman Ford //////////////////////////////////////////
--> For example, given the start location: Ralphs
            and given the destination: University Park
            the expected value: 0.77816 miles 
--> Time complexity: O (n x m)
--> Time taken: 4484 ms

![Shortest path](ubuntu/images/Shortestpath.JPG)

****************************************************************************************************
### Comparison of Dijkstra and Bellman Ford
*Dijkstra excels with speed, while Bellman-Ford ensures correctness at a 
higher computational cost*

This was expected having in mind the timecomplexity corresponding to each algorithm.
However, when to use each one depends on the circusntances of the problem, while Dijsktra (a Greedy algorithm) works only with non-negative edge weights,
Bellman Ford (a Dynamic programming algorithm) handles graphs with negative edge weights.

The following image shows the time taken of every function according to the destination.

![Comparison of Dijkstra and Bellman Ford](ubuntu/images/DijkstravsBellmanFord.JPG)

****************************************************************************************************
## Item 7 - Cycle detection
*Detect cycles using DFS in a subgraph*
--> For example, given the input for: 
            *left bound: -118.290
            *right bound: -118.275
            *upper bound: 34.032
            *lower bound: 34.011
            the expected value: there exists a cycle in the subgrapgh
--> Time complexity: O (n)
--> Time taken: 16 ms

See the image for reference
![ Cycle detection](ubuntu/images/CycleDetection.JPG)

****************************************************************************************************
## Item 8 - Topological Sort
The input are csv files with dependencies and then topological sort orders nodes based on 
the dependencies using BFS.
 **********Given************************
location_names = {"Ralphs", "Chick-fil-A", "KFC"}
dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"Chick-fil-A", "KFC"}}
Here, {"Ralphs","KFC"} means that Tommy must go to Ralphs prior to KFC.

--> For example, given an input a csv file with dependencies
               the expected output wouldbe the sorted locations for the given csv file
                                  : Grand & 30th
                                  : Grand & Adams
                                  : University Park
                                  : Jefferson/USC
--> Time complexity: O (n)
--> Time taken: 0 ms

![Topological Sort](ubuntu/images/TopologicalSort.JPG)

****************************************************************************************************
## Item 9 - Traveling Trojan
Provides brute force for correctness, backtracking for intermediate efficiency, and 2-opt/3-opt for 
scalability.

//////////////////// Brute force /////////////////////////////////////////////////////////////////////
--> For example, given the input: ... (see image below)
--> Time complexity: O (n x n!)
--> Time taken: 5 ms

//////////////////// Brute force w backtracking /////////////////////////////////////////////////////////
--> For example, given the input: ... (see image below)

--> Time complexity: O (n x n!)
--> Time taken: 3 ms

It is worth noting how the time to run the function using Brute force w backtracking decreased 66.67 %-
This lies in their approach to explore and solve problems. While both methods are exhaustive, backtracking 
introduces an optimization to systematically discard solutions that cannot possibly work, reducing unnecessary computation.

![Traveling Trojan](ubuntu/images/TravelingTrojan.JPG)

****************************************************************************************************
## Item 9-B - Travelling Trojan 2 opt and 3opt
//////////////////// 2 opt ///////////////////////////////////////////////////////////////
--> For example, given the input: path (see image below)
                 the expected result: 8.77622 miles
--> Time complexity: O ( n² )
--> Time taken: 0 ms

//////////////////// 3 opt ///////////////////////////////////////////////////////////////
--> For example, given the input: path (see image below)
                 the expected result: 8.77622 miles
--> Time complexity: O ( n³ )
--> Time taken: 1 ms

High level functions, in this case n**3, tend to be slower to solve, this is one reason 2 opt is faster.
A good practice learnt here is to try to avoid high order polynomios/functions if possible. 
If interesdted in seeing an animation of how Brute force, 2 opt and 3 opt animation works please see the Youtube video
 [youtube ref](https://www.youtube.com/watch?v=5uOlsDhjrDM&t=25s)

![Travelling Trojan 2 opt and 3opt](ubuntu/images/2optand3opt.JPG)

****************************************************************************************************
## Item 9-C Performance Benchmarks: Brute Force vs. Backtracking and 2-opt vs. 3-opt in TSP
Factorial growth of brute force O (n!) will eventually outpace the polynomial growth of 2-opt O (n²) and
3-opt O (n³) optimizations.

![ Performance Benchmarks](ubuntu/images/BruteForcevsBacktracking.JPG)

****************************************************************************************************
## Item 10 - Find Nearby
Operation: Filter locations within a radius and sort them by distance.
--> For example, given the input: 
                 1. attribute: fast_food
                 2. locations: Popeyes
                 3. radius: 32 
                 4. number: 3
    the expected result: 1 Five Guys
                         2 KFC
                         3 Honeybird
--> Time complexity: O (m log n)
--> Time taken: 5 ms

![Find Nearby](ubuntu/images/FindNearby.JPG)

****************************************************************************************************
## Item 11 - Trojan Path
This functions finds the shortest path and calculates the distance to visit all the locations given.

--> For example, given the input: ... see image below
              the expected distance of the path: 1.86368 miles
--> Time complexity: O ( n! )
--> Time taken: 385 ms

![Trojan Path](ubuntu/images/TrojanPath.JPG)

****************************************************************************************************
## Item 12 - Check Path
This functions verifies connectivity between two nodes under constraints such as gas tank size
to determine if current gas tank has enough fuel to take you to the desired location
--> For example, given the input: Food Mart
                     destination: Target
                      vol gas tank: 0.5
              expected result: Yes
--> Time complexity: O Q(n+m)
--> Time taken: 438 ms

![Check Path](ubuntu/images/CheckPath.JPG)

****************************************************************************************************
## Item 13 - Exit


# Conclusion
Working on advanced pathfinding algorithms like Dijkstra, Bellman-Ford, and tackling the Traveling 
Salesperson Problem with approaches such as brute force, backtracking, and 2-opt was challenging, nevertheless, we
gain a deeper understanding of graph theory. Along the way, it was helpful to reinforced class lectures and actually apply that 
knowledge to a hands-on project.
We were able to learn a lot from C++ theory , such as: recursion, pointers, and memory management and data structures too like adjacency lists and priority 
queues to make graph operations more efficient and used regex for dynamic location-based queries. Collaborating with
Git version control throughout the process ensured smooth teamwork and effective project management, making the 
experience both technically enriching and rewarding. 









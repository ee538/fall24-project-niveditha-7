#include "trojanmap.h"
#include <unordered_map>
#include <limits>

//-----------------------------------------------------
// TODO: Students should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return
 * -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string &id) { //using lookup method at and count to get Latt of thaat particular id 
  if(data.count(id)>0)
  {
    return data.at(id).lat;
  }
  else
    return -1;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist,
 * return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string &id) {
    if(data.count(id)>0)
  {
    return data.at(id).lon;
  }
  else
    return -1;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return
 * "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string &id) {
    if(data.count(id)>0)
  {
    return data.at(id).name;
  }
  else
    return "NULL";
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return
 * an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id) {
  if(data.find(id)!= data.end())
  {
    return data.at(id).neighbors;
  }
  return {}; //return empty if neigbours not found
}

/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 * The location name must be unique, which means there is only one node with the name.
 *
 * @param  {std::string} name          : location name
 * @return {std::string}               : id
 */
std::string TrojanMap::GetID(const std::string &name) {
  std::string res = "";
  for(const auto& node:data)
  {

    if (node.second.name ==  name)
    {
      res=node.first;
      break;
    }
  }
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not
 * exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) { //

std::pair<double, double> results(-1, -1);
if (name.empty()) {
    return results;
  }
std::string id = GetID(name);
  if(id!="")
  {
    results.first = GetLat(id);
    results.second= GetLon(id);
  }

  return results;
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * @param  {std::string} a          : first string
 * @param  {std::string} b          : second string
 * @return {int}                    : edit distance between two strings
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b) {     
    // strings to lowercase for comparison
    std::transform(a.begin(), a.end(), a.begin(), ::tolower);
    std::transform(b.begin(), b.end(), b.begin(), ::tolower);
    // If strings are equal, the distance is zero
    if (a == b) {
        return 0;  
    }
    // dimensions of DP matrix
    int m = a.size();  // # of rows 
    int n = b.size();  // # of columns
    // DP matrix, default value set to 0
    std::vector<std::vector<int>> dp(m + 1, std::vector<int>(n + 1, 0));
    // Populating first row
    for (int col = 0; col <= n; ++col) {
        dp[0][col] = col;
    }
    // Populating first column
    for (int row = 1; row <= m; ++row) {
        dp[row][0] = row;
    }
    // populating the matrix
    for (int i = 1; i <= m; ++i) {
        for (int j = 1; j <= n; ++j) {
            if (a[i - 1] == b[j - 1]) {
                // Characters match: no cost
                dp[i][j] = dp[i - 1][j - 1];
            } else {
                // Characters do not match: cost
                int insert_cost = dp[i][j - 1];    // Cost of inserting 
                int delete_cost = dp[i - 1][j];    // Cost of deleting 
                int replace_cost = dp[i - 1][j - 1];  // Cost of replacing
                dp[i][j] = 1 + std::min({insert_cost, delete_cost, replace_cost});
            }
        }
    }

    // return the edit distance
    return dp[m][n];
}

/**
 * FindClosestName: Given a location name, return the name with the smallest edit
 * distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : the closest name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = ""; // Start with a dummy word
  int smallest_distance = INT_MAX;  // Start with the largest possible distance

  // Iterate through all nodes in the map
  for (const auto& node : data) {
    const std::string& current_name = node.second.name;

    // Skip nodes with empty names
    if (current_name.empty()) {
      continue;
    }

    // Calculate the edit distance using the existing function
    int current_distance = CalculateEditDistance(name, current_name);

    // Update the closest name if a smaller distance is found
    if (current_distance < smallest_distance) {
      smallest_distance = current_distance;
      tmp = current_name;  // Use the original name for the result
    }
  }

  return tmp;
}



/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
   if (name.empty()) {
    return results;
  }
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  for(const auto& node:data)//traverse throough all  nodes 
  {
    std:: string nodeName = node.second.name;
    if(nodeName.size()<name.size()){continue;} //skip if the input name is longer than the present node
    std::transform(nodeName.begin(), nodeName.end(),nodeName.begin(), ::tolower);//convert the preseent nodettto lower case for comparison
    if(nodeName.substr(0, name.size())==name)//checking if the input name is present as substring in the found node.
    {
      results.push_back(node.second.name);
    }
  }
   
  return results;
}
//run time of auto complete -- answer : runtime of auto complete : Iterating through all the nodes -> O(n);
//converting each string to lower -> O(l) so combines run time complexity is O(n*l).

/**
 * GetAllCategories: Return all the possible unique location categories, i.e.
 * there should be no duplicates in the output.
 *
 * @return {std::vector<std::string>}  : all unique location categories
 */
std::vector<std::string> TrojanMap::GetAllCategories() {
  //creating a set to store unique categories
  std::set<std::string>uniqueCategories;
  //iterating all data in map
  for (const auto& entry : data){
    const auto& attributes =entry.second.attributes;
    //adding attributes to the set
    for (const auto& category: attributes){
      uniqueCategories.insert(category);
    }
  }
  //set to vector
  std::vector<std::string>result (uniqueCategories.begin(), uniqueCategories.end());
  return result;
}

/**
 * GetAllLocationsFromCategory: Return all the locations of the input category (i.e.
 * 'attributes' in data.csv). If there is no location of that category, return
 * (-1, -1). The function should be case-insensitive.
 *
 * @param  {std::string} category         : category name (attribute)
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetAllLocationsFromCategory(
    std::string category) {
  // initializing the vector    
  std::vector<std::string> res;
  // to lower case for case sensivity
  std::transform(category.begin(),category.end(), category.begin(), ::tolower);
  // iterating the map
  for (const auto&[location,node]: data){
    if (node.attributes.count(category)>0){
      res.push_back(location);
    }
  }
  return res;
}

/**
 * GetLocationRegex: Given the regular expression of a location's name, your
 * program should first check whether the regular expression is valid, and if so
 * it returns all locations that match that regular expression.
 *
 * @param  {std::regex} location name      : the regular expression of location
 * names
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetLocationRegex(std::regex location) {
  // initializing an empty vector
  std::vector<std::string> results;

  // iterating all entries in map
  for (auto & it:data){
    // ig name matches the regex, the location is added to vector
    if (std::regex_match(it.second.name,location)){
      results.push_back(it.first);
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 * We have provided the code for you. Please do not need to change this function.
 * You can use this function to calculate the distance between 2 nodes.
 * The distance is in mile.
 * The distance is calculated using the Haversine formula.
 * https://en.wikipedia.org/wiki/Haversine_formula
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id,
                                    const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) *
                                           cos(b.lat * M_PI / 180.0) *
                                           pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations
 * inside the vector.
 * We have provided the code for you. Please do not need to change this function.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++) {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path
 * which is a list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
 // for Reconstructing the path from goal to start
  std::vector<std::string> path;
  if(location1_name ==location2_name)
  {
    return path;
  }
  // Get unique IDs for the start and destination locations
  std::string start_id = TrojanMap::GetID(location1_name);
  std::string goal_id = TrojanMap::GetID(location2_name);

  // Priority queue to explore nodes in order of shortest known distance
  std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<>> pq;

  // Map to store the shortest distance and the corresponding path for each node
  std::unordered_map<std::string, double> distances;
  std::unordered_map<std::string, std::string> predecessors;

  // Initialize all distances to infinity
  for (const auto &node : data) {
    distances[node.first] = std::numeric_limits<double>::max();
  }

  // Distance to the start node is zero
  distances[start_id] = 0;
  pq.push({0, start_id});

  // Process nodes in the priority queue
  while (!pq.empty()) {
    auto [current_distance, current_id] = pq.top();
    pq.pop();

    // Stop if we reach the goal node
    if (current_id == goal_id) break;

    // Explore neighbors of the current node
    for (const auto &neighbor_id : data[current_id].neighbors) {
      double new_distance = current_distance + TrojanMap::CalculateDistance(current_id, neighbor_id);

      // If a shorter path to the neighbor is found, update it
      if (new_distance < distances[neighbor_id]) {
        distances[neighbor_id] = new_distance;
        predecessors[neighbor_id] = current_id;
        pq.push({new_distance, neighbor_id});
      }
    }
  }

  
  for (std::string at = goal_id; at != start_id; at = predecessors[at]) {
    if (predecessors.find(at) == predecessors.end()) {
      return {}; // No path found
    }
    path.push_back(at);
  }
  path.push_back(start_id);
  std::reverse(path.begin(), path.end());

  return path;
}



/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest
 * path which is a list of id. Hint: Do the early termination when there is no
 * change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
//initila checks similar to dijkstra's
  std::string start_location = GetID(location1_name);
  std::string end_location = GetID(location2_name);

  if (data.find(start_location) == data.end() || data.find(end_location) == data.end()) {
      return {};  // Return an empty path if either location is missing
  }

  // Initialize distances to infinity and set the start node's distance to 0
  std::unordered_map<std::string, double> short_distance_from_start;
  std::unordered_map<std::string, std::string> previous;

  for (auto &node : data) {
    short_distance_from_start[node.first] = INT_MAX;
  }
  short_distance_from_start[start_location] = 0;

  int V = data.size();
  bool updated;

  // Perform V-1 iterations of relaxation
  for (int i = 0; i < V - 1; i++) {
    updated = false;

    // For each node, check all its neighbors
    for (auto &node : data) {
      std::string u = node.first;

      if (short_distance_from_start[u] == INT_MAX) continue; // Skip unreachable nodes

      for (auto &v : GetNeighborIDs(u)) {
        double weight = CalculateDistance(u, v);

        // Relax the edge (u, v)
        if (short_distance_from_start[u] + weight < short_distance_from_start[v]) {
          short_distance_from_start[v] = short_distance_from_start[u] + weight;
          previous[v] = u;
          updated = true; //since we are updating the distance by checking the shortest path
        }
      }
    }

    // If no update was made during this iteration, we terminate early
    if (!updated) break;
  }

  // If end_location is still at infinity, no path was found
  if (short_distance_from_start[end_location] == INT_MAX) {
    return {};  // No path exists
  }

  // Recoverring the path from end_location to start_location
  for (std::string at = end_location; at != ""; at = previous[at]) {
    path.push_back(at);
    if (at == start_location) break;
  }

  // Reverse to get the path from start to end
  std::reverse(path.begin(), path.end());

  // Verify that the path starts with start_location
  if (path.empty() || path[0] != start_location) {
    return {};  // Return empty if no valid path exists
  }
  return path;
}


// //non-optimised bellman ford
// std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
//     std::string location1_name, std::string location2_name) {
//   std::vector<std::string> path;

//   // Get the IDs for the start and end locations
//   std::string start_location = GetID(location1_name);
//   std::string end_location = GetID(location2_name);

//   // Check if start or end location exists in the map
//   if (data.find(start_location) == data.end() || data.find(end_location) == data.end()) {
//     return {};  // Return an empty path if either location is missing
//   }

//   // Initialize distances to infinity and set the start node's distance to 0
//   std::unordered_map<std::string, double> short_distance_from_start;
//   std::unordered_map<std::string, std::string> prev_distance;

//   for (auto &node : data) {
//     short_distance_from_start[node.first] = INT_MAX;
//   }
//   short_distance_from_start[start_location] = 0;

//   int V = data.size();

//   // Perform exactly V-1 iterations of relaxation
//   for (int i = 0; i < V - 1; i++) {
//     // For each node, check all its neighbors
//     for (auto &node : data) {
//       std::string u = node.first;

//       if (short_distance_from_start[u] == INT_MAX) continue; // Skip unreachable nodes

//       for (auto &v : GetNeighborIDs(u)) {
//         double weight = CalculateDistance(u, v);

//         // Relax the edge (u, v)
//         if (short_distance_from_start[u] + weight < short_distance_from_start[v]) {
//           short_distance_from_start[v] = short_distance_from_start[u] + weight;
//           prev_distance[v] = u;
//         }
//       }
//     }
//   }

//   // If end_location is still at infinity, no path was found
//   if (short_distance_from_start[end_location] == INT_MAX) {
//     return {};  // No path exists
//   }

//   // Reconstruct the path from end_location to start_location
//   for (std::string at = end_location; at != ""; at = prev_distance[at]) {
//     path.push_back(at);
//     if (at == start_location) break;
//   }

//   // Reverse to get the path from start to end
//   std::reverse(path.begin(), path.end());

//   // Verify that the path starts with start_location
//   if (path.empty() || path[0] != start_location) {
//     return {};  // Return empty if no valid path exists
//   }

//   return path;
// }





/**
 * Traveling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path, 
 *                                                                      for example: {10.3, {{0, 1, 2, 3, 4, 0}, {0, 1, 2, 3, 4, 0}, {0, 4, 3, 2, 1, 0}}},
 *                                                                      where 10.3 is the total distance, 
 *                                                                      and the first vector is the path from 0 and travse all the nodes and back to 0,
 *                                                                      and the second vector is the path shorter than the first one,
 *                                                                      and the last vector is the shortest path.
 */
// Please use brute force to implement this function, ie. find all the permutations.
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  // Handle the edge case where there are no locations
  if (location_ids.empty()) {
    return records;
  }
  
  // Initialize minimum distance and optimal path
  double min_distance = std::numeric_limits<double>::max();
  std::vector<std::string> optimal_path;

  // Generate all permutations of the locations (excluding the first location as the start point)
  std::vector<std::string> perm_locations = location_ids;
  perm_locations.erase(perm_locations.begin());
  std::sort(perm_locations.begin(), perm_locations.end());

  do {
    // Construct the full path (start -> permutation -> start)
    std::vector<std::string> current_path = {location_ids[0]};
    current_path.insert(current_path.end(), perm_locations.begin(), perm_locations.end());
    current_path.push_back(location_ids[0]);

    // Calculate the distance of the current path
    double current_distance = CalculatePathLength(current_path);

    // Store the path in records
    records.second.push_back(current_path);

    // Updating the minimum distance and the optimal path if this path is shorter
    if (current_distance < min_distance) {
      min_distance = current_distance;
      optimal_path = current_path;
    }

  } while (std::next_permutation(perm_locations.begin(), perm_locations.end())); //https://cplusplus.com/reference/algorithm/next_permutation/

  // Setting the minimum distance in records
  records.first = min_distance;

  // Adding the optimal path to the records
  records.second.push_back(optimal_path);
  return records;
}

// Please use backtracking to implement this function
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  // Handle the edge case where there are no locations
  if (location_ids.empty()) {
    return records;
  }

  // Initialize variables
  double min_distance = std::numeric_limits<double>::max(); // Minimum distance found
  std::vector<std::string> optimal_path;                   // Stores the shortest path
  std::vector<std::string> current_path;                   // Path being explored
  current_path.push_back(location_ids[0]);                 // Start at the first location

  // Helper function to perform backtracking
  auto backtrack = [&](auto&& backtrack, std::string current_node, double current_distance) -> void {
    // If all locations are visited, calculate the round trip and update the result
    if (current_path.size() == location_ids.size()) {
      double round_trip_distance = current_distance + CalculateDistance(current_node, location_ids[0]);
      current_path.push_back(location_ids[0]); // Complete the round trip
      records.second.push_back(current_path); // Store the completed path
      
      if (round_trip_distance < min_distance) {
        min_distance = round_trip_distance;
        optimal_path = current_path;
      }
      current_path.pop_back(); // Backtrack to remove the round trip
      return;
    }

    // Explore all unvisited locations
    for (const auto& next_node : location_ids) {
      if (std::find(current_path.begin(), current_path.end(), next_node) == current_path.end()) {
        current_path.push_back(next_node); // Visit the next location
        double distance_to_next = CalculateDistance(current_node, next_node);
        backtrack(backtrack, next_node, current_distance + distance_to_next);
        current_path.pop_back(); // Backtrack to explore a different path
      }
    }
  };

  // Start backtracking from the first location
  backtrack(backtrack, location_ids[0], 0.0);

  // Store the minimum distance and optimal path
  records.first = min_distance;
  records.second.push_back(optimal_path);
  return records;
  
}

// Hint: https://en.wikipedia.org/wiki/2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;

   // Handle edge cases: no locations or a single location
    if (location_ids.empty()) {
        return records;
    }
    if (location_ids.size() == 1) {
        double distance = 0.0;
        records.first = distance;
        records.second.push_back({location_ids[0], location_ids[0]}); // Round trip
        return records;
    }

    // Initialize the path and calculate its initial length
    std::vector<std::string> current_path = location_ids;
    current_path.push_back(location_ids[0]); // Make it a round trip
    double best_distance = CalculatePathLength(current_path);

    // Store the initial path
    records.second.push_back(current_path);
    records.first = best_distance;

    bool improved = true; // Flag to track if improvements are being made

    // Repeat until no further improvement is possible
    while (improved) {
        improved = false;

        // Iterate over all pairs of edges to swap
        for (size_t i = 1; i < current_path.size() - 2; ++i) {
            for (size_t j = i + 1; j < current_path.size() - 1; ++j) {
                
                // Generate a new path by swapping edges between i and j
                std::vector<std::string> new_path = current_path;
                std::reverse(new_path.begin() + i, new_path.begin() + j + 1);

                // Calculate the distance of the new path
                double new_distance = CalculatePathLength(new_path);

                // Update if the new path is better
                if (new_distance < best_distance) {
                    current_path = new_path;
                    best_distance = new_distance;
                    records.second.push_back(current_path); // Save progress
                    records.first = best_distance;
                    improved = true; // Continue optimizing
                    break; // Restart the loop to apply further swaps
                }
            }
            if (improved) break;
        }
    }
  return records;
}

// This is optional
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_3opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
   

    // Handle edge cases: no locations or a single location
    if (location_ids.size() <= 1) {
        double distance = 0.0;
        if (!location_ids.empty()) {
            records.second.push_back({location_ids[0], location_ids[0]}); // Round trip
        }
        records.first = distance;
        return records;
    }

    // Initialize the path and calculate its initial length
    std::vector<std::string> best_path = location_ids;
    best_path.push_back(location_ids[0]); // Make it a round trip
    double best_distance = CalculatePathLength(best_path);

    // Store the initial path
    records.second.push_back(best_path);
    records.first = best_distance;

    bool improvement = true;

    // Optimization loop
    while (improvement) {
        improvement = false;

        // Iterate over all possible triplets of indices (i, j, k)
        for (int i = 1; i < best_path.size() - 3; ++i) {
            for (int j = i + 1; j < best_path.size() - 2; ++j) {
                for (int k = j + 1; k < best_path.size() - 1; ++k) {
                    
                    // Perform 3-opt swaps and evaluate all possible configurations
                    std::vector<std::vector<std::string>> candidates = Generate3OptCandidates(best_path, i, j, k);

                    for (const auto& new_path : candidates) {
                        double new_distance = CalculatePathLength(new_path);

                        // Update if the new path is better
                        if (new_distance < best_distance) {
                            best_path = new_path;
                            best_distance = new_distance;
                            records.second.push_back(best_path); // Track progress
                            improvement = true;                  // Continue optimizing
                        }
                    }

                    if (improvement) break; // Restart the loops if an improvement is found
                }
                if (improvement) break;
            }
            if (improvement) break;
        }
    }

    records.first = best_distance; // Store the best distance
    return records;
}

std::vector<std::vector<std::string>> TrojanMap::Generate3OptCandidates(
    const std::vector<std::string>& path, int i, int j, int k) {

    std::vector<std::vector<std::string>> candidates;

    // Original path
    candidates.push_back(path);

    // Case 1: Reverse [i+1, j]
    std::vector<std::string> case1 = path;
    std::reverse(case1.begin() + i + 1, case1.begin() + j + 1);
    candidates.push_back(case1);

    // Case 2: Reverse [j+1, k]
    std::vector<std::string> case2 = path;
    std::reverse(case2.begin() + j + 1, case2.begin() + k + 1);
    candidates.push_back(case2);

    // Case 3: Reverse both [i+1, j] and [j+1, k]
    std::vector<std::string> case3 = path;
    std::reverse(case3.begin() + i + 1, case3.begin() + j + 1);
    std::reverse(case3.begin() + j + 1, case3.begin() + k + 1);
    candidates.push_back(case3);

    // Case 4: Reverse [i+1, k]
    std::vector<std::string> case4 = path;
    std::reverse(case4.begin() + i + 1, case4.begin() + k + 1);
    candidates.push_back(case4);

    return candidates;
}


/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_locations.csv"
 *   File content:
 *    Name
 *    Ralphs
 *    KFC
 *    Chick-fil-A
 *   Output: ['Ralphs', 'KFC', 'Chick-fil-A']
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(
    std::string locations_filename) {
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, word)) {
    location_names_from_csv.push_back(word);
  }
  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_dependencies.csv"
 *   File content:
 *     Source,Destination
 *     Ralphs,Chick-fil-A
 *     Ralphs,KFC
 *     Chick-fil-A,KFC
 *   Output: [['Ralphs', 'Chick-fil-A'], ['Ralphs', 'KFC'], ['Chick-fil-A', 'KFC']]
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(
    std::string dependencies_filename) {
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    std::vector<std::string> dependency;
    while (getline(s, word, ',')) {
      dependency.push_back(word);
    }
    dependencies_from_csv.push_back(dependency);
  }
  fin.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a
 * sorting of nodes that satisfies the given dependencies. If there is no way to
 * do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(
    std::vector<std::string> &locations,
    std::vector<std::vector<std::string>> &dependencies) {
  std::unordered_map<std::string, int> in_degrees;
  std::unordered_map<std::string, std::vector<std::string>> graph;

  // Building graph and degree
  for (const auto &location : locations) {
    in_degrees[location] = 0;
    graph[location] = {};
  }
  for (const auto &dependency : dependencies) {
    graph[dependency[0]].push_back(dependency[1]);
    in_degrees[dependency[1]]++;
  }
  // nodes with no dependencies
  std::queue<std::string> q;
  for (const auto &[node, degree] : in_degrees) {
    if (degree == 0) q.push(node);
  }
  
  // Topological Sort
  std::vector<std::string> result;
  while (!q.empty()) {
    auto node = q.front();
    q.pop();
    result.push_back(node);
    for (const auto &neighbor : graph[node]) {
      if (--in_degrees[neighbor] == 0) q.push(neighbor);
    }
  }
  // all nodes  sorted ??
  return (result.size() == locations.size()) ? result : std::vector<std::string>{};
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  if (data.find(id) == data.end()) return false;
  double lon = GetLon(id); // Get longitude
  double lat = GetLat(id); // Get latitude

  // Checking if the point is within the square bounds
  return (lon >= square[0] && lon <= square[1] && lat <= square[2] && lat >= square[3]);
  return true;
  return true;
}


/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location
 * ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square
 * area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the
 * square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  // Iterating over all nodes in the g raph
  for (const auto &node : data) {
    if (inSquare(node.first, square)) {
      subgraph.push_back(node.first); // Add nodes within the square to the subgraph
    }
  }


  return subgraph;
}
bool DFS_CycleDetection(const std::string &node, const std::string &parent,
                        std::unordered_map<std::string, bool> &visited,
                        std::unordered_map<std::string, std::vector<std::string>> &subgraph_map) {
  visited[node] = true;

  for (const auto &neighbor : subgraph_map[node]) {
    if (!visited[neighbor]) {
      if (DFS_CycleDetection(neighbor, node, visited, subgraph_map)) {
        return true; // Cycle detected in a recursive call
      }
    } else if (neighbor != parent) {
      return true; // when Back edge found (cycle detected)
    }
  }

  return false; // No cycle is detected
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true
 * if there is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the
 * square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  // Create an adjacency list for the subgraph
  std::unordered_map<std::string, std::vector<std::string>> subgraph_map;

  for (const auto &node : subgraph) {
    for (const auto &neighbor : GetNeighborIDs(node)) {
      if (inSquare(neighbor, square)) {
        subgraph_map[node].push_back(neighbor); // Adding the  edge only if neighbor is in the square
      }
    }
  }

  // Initialize visited map
  std::unordered_map<std::string, bool> visited;

  for (const auto &node : subgraph) {
    visited[node] = false; // Set all nodes as unvisited
  }

  // Perform DFS for cycle detection
  for (const auto &node : subgraph) {
    if (!visited[node]) {
      if (DFS_CycleDetection(node, "", visited, subgraph_map)) {
        return true; // Cycle detected
      }
    }
  }

  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and
 * return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {double} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
   // Validate if the location exists in the map
  std::string location_id = GetID(name);
  if (location_id.empty()) {
    std::cout << "Error: Location not found - " << name << std::endl;
    return res;  // If the location does not exist, return an empty result
  }

  // Retrieve all locations with the specified attribute
  auto matching_ids = GetAllLocationsFromCategory(attributesName);
  if (matching_ids.empty()) {
    std::cout << "Error: No locations found with attribute - " << attributesName << std::endl;
    return res;
  }

  // Vector to store distances and matching locations
  std::vector<std::pair<double, std::string>> candidates;

  for (const auto& id : matching_ids) {
    if (id == location_id) continue;  // Skip the given location itself

    // Calculate the distance using the IDs
    double distance = CalculateDistance(location_id, id);

    // Include only those within the specified radius
    if (distance <= r) {
      candidates.push_back({distance, id});
    }
  }

  // Sorting the candidates by distance (nearest to farthest)
  std::sort(candidates.begin(), candidates.end());

  // Adding  up to `k` closest locations to the result
  for (int i = 0; i < std::min(k, (int)candidates.size()); ++i) {
    res.push_back(candidates[i].second);  // Add the ID of the location
  }
  return res;
}




/**
 * Shortest Path to Visit All Nodes: Given a list of locations, return the shortest
 * path which visit all the places and no need to go back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::vector<std::string> }      : the shortest path
 */
std::vector<std::string> TrojanMap::TrojanPath(
      std::vector<std::string> &location_names) {
    std::vector<std::string> res;
    // Handle empty input case
  if (location_names.empty()) {
    std::cerr << "Error: Input location list is empty!" << std::endl;
    return res;
  }
// Convert location names to unique IDs
  std::vector<std::string> location_ids;
  for (const auto &location_name : location_names) {
    location_ids.push_back(TrojanMap::GetID(location_name));
  }

  // Precompute distances and paths between all pairs of locations
  std::map<std::pair<std::string, std::string>, double> distance_map; // Stores pairwise distances
  std::map<std::pair<std::string, std::string>, std::vector<std::string>> path_map; // Stores pairwise shortest paths

  for (const auto &source_id : location_ids) {
    for (const auto &destination_id : location_ids) {
      if (source_id != destination_id) {
        // Compute shortest path and its distance
        auto shortest_path = TrojanMap::CalculateShortestPath_Dijkstra(TrojanMap::GetName(source_id), TrojanMap::GetName(destination_id));
        path_map[{source_id, destination_id}] = shortest_path;
        distance_map[{source_id, destination_id}] = TrojanMap::CalculatePathLength(shortest_path);
      }
    }
  }

  // Use backtracking to find the optimal path visiting all locations
  double shortest_total_distance = std::numeric_limits<double>::max();
  std::vector<std::string> current_path, best_path;
  std::set<std::string> visited_locations;

  SolveTSPWithBacktracking(current_path, visited_locations, location_ids, best_path, shortest_total_distance, distance_map);

  // Construct the full path using the pairwise shortest paths
  for (size_t i = 0; i < best_path.size() - 1; ++i) {
    const auto &segment = path_map[{best_path[i], best_path[i + 1]}];
    res.insert(res.end(), segment.begin(), segment.end());

    // Avoid duplicating nodes between consecutive segments
    if (i != best_path.size() - 2) {
      res.pop_back();
    }
  }

  return res; // Return the final shortest path

  
}


void TrojanMap::SolveTSPWithBacktracking(std::vector<std::string> &current_path,
                                         std::set<std::string> &visited,
                                         const std::vector<std::string> &all_locations,
                                         std::vector<std::string> &best_path,
                                         double &shortest_distance,
                                         const std::map<std::pair<std::string, std::string>, double> &distance_map) {
  // Base case: If all locations are visited
  if (visited.size() == all_locations.size()) {
    double current_distance = 0;

    // Calculate the total distance of the current path
    for (size_t i = 0; i < current_path.size() - 1; i++) {
      // Fetch the distance directly from the map
      auto it = distance_map.find({current_path[i], current_path[i + 1]});
      if (it != distance_map.end()) {
        current_distance += it->second;
      } else {
        // If distance is not precomputed, assume an infinitely large value
        current_distance += std::numeric_limits<double>::max();
      }
    }

    // Update the shortest distance and best path if the current path is shorter
    if (current_distance < shortest_distance) {
      best_path = current_path;
      shortest_distance = current_distance;
    }
    return;
  }

  // Recursive exploration of paths
  for (const auto &location_id : all_locations) {
    if (visited.find(location_id) == visited.end()) {
      // Add the location to the current path and mark it as visited
      visited.insert(location_id);
      current_path.push_back(location_id);

      // Recurse to explore further paths
      SolveTSPWithBacktracking(current_path, visited, all_locations, best_path, shortest_distance, distance_map);

      // Backtrack to explore alternative paths
      visited.erase(location_id);
      current_path.pop_back();
    }
  }
}



/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>>& q) {
    std::vector<bool> ans(q.size());

    // processing each query independently
    for (size_t i = 0; i < q.size(); ++i) {
        double gas_tank = q[i].first;
        const auto &locations = q[i].second;

        if (locations.size() != 2) {
            ans[i] = false; // invalid format
            continue;
        }

        std::string start_id = GetID(locations[0]);
        std::string end_id = GetID(locations[1]);

        if (start_id.empty() || end_id.empty()) {
            ans[i] = false; // 1/2 locations do not exist
            continue;
        }

        // union-find structure based on the gas tank size
        std::unordered_map<std::string, std::string> parent;
        for (const auto &node : data) {
            parent[node.first] = node.first; // initializing each node as its own parent
        }

        for (const auto &node : data) {
            const auto &neighbors = GetNeighborIDs(node.first);
            for (const auto &neighbor : neighbors) {
                double distance = CalculateDistance(node.first, neighbor);

                // union  nodes if the distance is <= gas tank size
                if (distance <= gas_tank) {
                    Unite(parent, node.first, neighbor);
                }
            }
        }

        // verifying if start/end nodes are in the same connected component
        ans[i] = (Find(parent, start_id) == Find(parent, end_id));
    }

    return ans;
}

void TrojanMap::InteractivePathQuery() {
    std::vector<std::pair<double, std::vector<std::string>>> queries;

    while (true) {
        std::string start_location, destination;
        double gas_tank;

        //  start location
        std::cout << "Please input the start location: ";
        std::getline(std::cin, start_location);

        //  destination
        std::cout << "Please input the destination: ";
        std::getline(std::cin, destination);

        // input gas tank size
        std::cout << "Please input the volume of the gas tank: ";
        std::cin >> gas_tank;
        std::cin.ignore(); 

        // save the query
        queries.push_back({gas_tank, {start_location, destination}});

        // more queries are needed??
        std::cout << "More Query? (y/n): ";
        char more_query;
        std::cin >> more_query;
        std::cin.ignore(); 

        if (more_query == 'n' || more_query == 'N') {
            break;
        }
    }

    // process the queries
    std::vector<bool> results = Queries(queries);

    // output
    std::cout << "***Results****" << std::endl;
    for (size_t i = 0; i < queries.size(); ++i) {
        const auto &query = queries[i];
        const std::string &start_location = query.second[0];
        const std::string &destination = query.second[1];
        double gas_tank = query.first;

        std::cout << "From " << start_location << " to " << destination
                  << " with " << gas_tank << " gallons of gas tank: "
                  << (results[i] ? "Yes" : "No") << std::endl;
    }
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * We have provided the code for you. Please do not need to change this function.
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0])) n.attributes.insert(word);
        if (isdigit(word[0])) n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

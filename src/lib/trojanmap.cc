#include "trojanmap.h"
#include <unordered_map>
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
  
  std::vector<std::string> path;// to store the final path
  if(location1_name ==location2_name)
  {
    return path;
  }
  //gettig IDs for start and end location
  std::string start_location = GetID(location1_name);
  std::string end_location = GetID(location2_name);
  if (data.find(start_location) == data.end() || data.find(end_location) == data.end()) {
        std::cout << "One or both locations do not exist in the map." << std::endl;
        return {};  // Return an empty path if either location is missing
  }

  //creating an unordered map to store the shortest path from start nodes
  std::unordered_map<std::string, double> short_distance_from_start;

  //unordered map to store the previous distance 
  std::unordered_map<std::string, std::string> prev_distance;
  //unordered map to store the visited nodes
  std::unordered_map<std::string, double> visited_nodes;

  //auto initialising all the distances initially to infinity and setting visited to false
  for(auto &node:data)
  {
    short_distance_from_start[node.first] = INT_MAX;
    visited_nodes[node.first] = false;
  }
  //Set up the priority queue with pairs of distance(double) and node ID(string), and store it in vector format with min-heap structure
  std::priority_queue<std::pair<double, std::string>,      
  std::vector<std::pair<double, std::string>>, std::greater<std::pair<double, std::string>>> min_distance_queue;
  // Initialize the starting node's distance to 0 and add it to the priority queue
    short_distance_from_start[start_location] = 0; // to itself will be zero 
    min_distance_queue.push({0, start_location});
     // Begin Dijkstra's algorithm
    while (!min_distance_queue.empty()) {
        std::string current = min_distance_queue.top().second; //second gives us node ID
        min_distance_queue.pop();

        // If the node has been visited, skip it
        if (visited_nodes[current]) continue;

        // Mark the current node as visited
        visited_nodes[current] = true;

        // If we've reached the goal, break out of the loop(if currentID is eequal to end ID)
        if (current == end_location) break;

        // For each neighbor of the current node
        for (auto &neighbor : GetNeighborIDs(current)) {
            // Calculate distance to the neighbor
            double weight = CalculateDistance(current, neighbor);
            
            // Relaxation step: check if a shorter path is found
            if (short_distance_from_start[current] + weight < short_distance_from_start[neighbor]) {
                short_distance_from_start[neighbor] = short_distance_from_start[current] + weight;
                prev_distance[neighbor] = current;
                min_distance_queue.push({short_distance_from_start[neighbor], neighbor});
            }
        }
    }

    // once the loop ends, trace back the path from goal to start using the `previous` map until it reaches start location
    for (std::string at = end_location; at != ""; at = prev_distance[at]) {
        path.push_back(at); //each node is being added to the path vector
        if (at == start_location) break;
    }

    // Reverse to get the path from start to goal
    std::reverse(path.begin(), path.end()); //since nodes are stored in reverse order in path

    // If the start node is not in path, it means no path was found
    if (path.empty() || path[0] != start_location) {
        return {};  // Return empty if no valid path exists
    }

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
  return records;
}

// Please use backtracking to implement this function
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// Hint: https://en.wikipedia.org/wiki/2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// This is optional
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_3opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
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
  std::vector<std::string> result;
  return result;     
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
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
  return subgraph;
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
    return res;
}

/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>>& q) {
    std::vector<bool> ans(q.size());
    return ans;
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

#include "src/lib/trojanmap.h"
#include <benchmark/benchmark.h>
#include <string>
#include <vector>

// Benchmark for Dijkstra's Algorithm
static void BM_Dijkstra(benchmark::State& state) {
    TrojanMap map;

    // Example: Add pairs of start and end node names for different input sizes
    std::vector<std::pair<std::string, std::string>> node_pairs = {
        {"USC Village Gym", "Target"},  // Small input
        {"West Vernon Elementary School", "Vermont Elementary School"},             // Medium input
        {"Ralphs", "Chase"}  // Large input
    };

    for (auto _ : state) {
        int index = state.range(0) % node_pairs.size();  // Choose a pair based on range
        map.CalculateShortestPath_Dijkstra(node_pairs[index].first, node_pairs[index].second);
    }
}
BENCHMARK(BM_Dijkstra)->Range(1, 3);  // Sweeping over the node pairs

// Benchmark for Bellman-Ford Algorithm
static void BM_BellmanFord(benchmark::State& state) {
    TrojanMap map;

    // Example: Add pairs of start and end node names for different input sizes
    std::vector<std::pair<std::string, std::string>> node_pairs = {
        {"USC Village Gym", "Target"},  // Small input
        {"West Vernon Elementary School", "Vermont Elementary School"},             // Medium input
        {"Ralphs", "Chase"}  // Large input
    };

    for (auto _ : state) {
        int index = state.range(0) % node_pairs.size();  // Choose a pair based on range
        map.CalculateShortestPath_Bellman_Ford(node_pairs[index].first, node_pairs[index].second);
    }
}
BENCHMARK(BM_BellmanFord)->Range(1, 3);  // Sweeping over the node pairs



// Benchmark for Autocomplete
static void BM_Autocomplete(benchmark::State& state) {
    TrojanMap map;
    std::string prefix = "Ch"; // Example prefix to test autocomplete functionality

    for (auto _ : state) {
        map.Autocomplete(prefix);
    }
}
BENCHMARK(BM_Autocomplete)->RangeMultiplier(2)->Range(2, 100);

static void BM_FindNearby(benchmark::State& state) {
    TrojanMap map;
    std::string category = "restaurant";  // Example category
    std::string location_name = "Chick-fil-A";  // Example starting location
    double radius = state.range(0);  // Sweep the radius parameter
    int k = 10;  // Top k results

    for (auto _ : state) {
        map.FindNearby(category, location_name, radius, k);
    }
}
BENCHMARK(BM_FindNearby)->RangeMultiplier(2)->Range(1, 50);


BENCHMARK_MAIN();

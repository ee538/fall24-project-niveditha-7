#include "src/lib/trojanmap.h"
#include <benchmark/benchmark.h>
#include <vector>


static void BM_TSP_BruteForce(benchmark::State& state) {
    TrojanMap map;
    std::vector<std::string> location_ids(state.range(0), "sample_location");
    for (auto _ : state) {
        auto result = map.TravelingTrojan_Brute_force(location_ids);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_TSP_BruteForce)->DenseRange(2, 10, 1);

static void BM_TSP_Backtracking(benchmark::State& state) {
    TrojanMap map;
    std::vector<std::string> location_ids(state.range(0), "sample_location");
    for (auto _ : state) {
        auto result = map.TravelingTrojan_Backtracking(location_ids);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_TSP_Backtracking)->DenseRange(2, 10, 1);


static void BM_TSP_2opt(benchmark::State& state) {
    TrojanMap map;
    std::vector<std::string> location_ids(state.range(0), "sample_location");
    for (auto _ : state) {
        auto result = map.TravelingTrojan_2opt(location_ids);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_TSP_2opt)->DenseRange(2, 10, 1);

static void BM_TSP_3opt(benchmark::State& state) {
    TrojanMap map;
    std::vector<std::string> location_ids(state.range(0), "sample_location");
    for (auto _ : state) {
        auto result = map.TravelingTrojan_3opt(location_ids);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_TSP_3opt)->DenseRange(2, 10, 1);

// Benchmark main function
BENCHMARK_MAIN();

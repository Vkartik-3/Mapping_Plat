// Copyright (C) 2025 Kartik Vadhawana
//
// nanoflann k-d tree benchmarks: build over 100k points, and 1000 nearest-
// neighbour queries on a pre-built tree.

#include <benchmark/benchmark.h>

#include <random>

#include "geometry/index/kdtree.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;

static void kdtree_build_100k(benchmark::State &state) {
    auto frame = util::synthetic::lidar_frame(80000, 20000, -1.7f, 0, 5); // 100k pts
    for (auto _ : state) {
        geometry::index::KDTree3D tree(frame);
        benchmark::DoNotOptimize(tree.size());
    }
    state.SetItemsProcessed(state.iterations() * frame.point_count);
}
BENCHMARK(kdtree_build_100k)->Unit(benchmark::kMillisecond);

static void kdtree_nn_1k(benchmark::State &state) {
    auto frame = util::synthetic::lidar_frame(80000, 20000, -1.7f, 0, 5);
    geometry::index::KDTree3D tree(frame);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> q(-60.0f, 60.0f);
    for (auto _ : state) {
        for (int i = 0; i < 1000; ++i) {
            bool found;
            auto hit = tree.nearest_neighbor(q(rng), q(rng), q(rng), found);
            benchmark::DoNotOptimize(hit.index);
        }
    }
    state.SetItemsProcessed(state.iterations() * 1000);
}
BENCHMARK(kdtree_nn_1k)->Unit(benchmark::kMillisecond);

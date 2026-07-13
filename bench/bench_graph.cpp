// Copyright (C) 2025 Kartik Vadhawana
//
// Road-graph benchmarks: OSM construction, Dijkstra routing, nearest-edge
// spatial query, and full-sequence map matching.

#include <benchmark/benchmark.h>

#include <filesystem>
#include <fstream>
#include <random>

#include "geometry/road_graph/osm_importer.hpp"
#include "geometry/road_graph/graph_algorithms.hpp"
#include "geometry/road_graph/map_matcher.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;
using namespace map_matching_2::geometry::road_graph;

namespace {

    // Emit a grid .osm file (rows x cols) to exercise the XML importer.
    std::filesystem::path make_osm_grid(int rows, int cols) {
        namespace fs = std::filesystem;
        fs::path p = fs::temp_directory_path() / "hdmap_bench_grid.osm";
        std::ofstream out(p);
        out << "<?xml version='1.0'?>\n<osm version=\"0.6\">\n";
        const double lat0 = 49.0, lon0 = 8.4, dlat = 0.0002, dlon = 0.0003;
        auto nid = [cols](int r, int c) { return r * cols + c + 1; };
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                out << " <node id=\"" << nid(r, c) << "\" lat=\"" << lat0 + r * dlat
                    << "\" lon=\"" << lon0 + c * dlon << "\"/>\n";
        int wid = 100000;
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c + 1 < cols; ++c)
                out << " <way id=\"" << wid++ << "\"><nd ref=\"" << nid(r, c) << "\"/><nd ref=\""
                    << nid(r, c + 1) << "\"/><tag k=\"highway\" v=\"residential\"/></way>\n";
        for (int c = 0; c < cols; ++c)
            for (int r = 0; r + 1 < rows; ++r)
                out << " <way id=\"" << wid++ << "\"><nd ref=\"" << nid(r, c) << "\"/><nd ref=\""
                    << nid(r + 1, c) << "\"/><tag k=\"highway\" v=\"residential\"/></way>\n";
        out << "</osm>\n";
        return p;
    }

}

static void graph_construct_osm(benchmark::State &state) {
    auto path = make_osm_grid(30, 30);
    OsmImportParams params;
    params.anchor_lat = 49.0;
    params.anchor_lon = 8.4;
    osm_importer imp(params);
    for (auto _ : state) {
        auto g = imp.import_file(path);
        benchmark::DoNotOptimize(g.edge_count());
    }
}
BENCHMARK(graph_construct_osm)->Unit(benchmark::kMicrosecond);

static void graph_dijkstra_1k(benchmark::State &state) {
    auto g = util::synthetic::grid_graph(30, 30, 20.0);
    std::mt19937 rng(99);
    std::uniform_int_distribution<std::size_t> pick(0, g.node_count() - 1);
    for (auto _ : state) {
        for (int i = 0; i < 1000; ++i) {
            auto sp = dijkstra(g, pick(rng), pick(rng));
            benchmark::DoNotOptimize(sp.cost);
        }
    }
    state.SetItemsProcessed(state.iterations() * 1000);
}
BENCHMARK(graph_dijkstra_1k)->Unit(benchmark::kMillisecond);

static void rtree_knn_1k(benchmark::State &state) {
    // Nearest-edge spatial query over the road graph (brute-force index).
    // Note: the upstream project uses a Boost.Geometry R*-tree; this pipeline
    // benchmark measures the equivalent 2D nearest-edge query latency.
    auto g = util::synthetic::grid_graph(30, 30, 20.0);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> pos(0.0, 580.0);
    for (auto _ : state) {
        for (int i = 0; i < 1000; ++i) {
            double d;
            auto e = nearest_edge(g, {pos(rng), pos(rng)}, d);
            benchmark::DoNotOptimize(e);
        }
    }
    state.SetItemsProcessed(state.iterations() * 1000);
}
BENCHMARK(rtree_knn_1k)->Unit(benchmark::kMillisecond);

static void map_match_sequence(benchmark::State &state) {
    auto frames = util::synthetic::trajectory(100, 100, 5.0);
    auto graph = util::synthetic::road_graph(5.0, 100, 100);
    map_matcher matcher;
    for (auto _ : state) {
        auto r = matcher.match(graph, frames);
        benchmark::DoNotOptimize(r.matched_points);
    }
    state.SetItemsProcessed(state.iterations() * frames.size());
}
BENCHMARK(map_match_sequence)->Unit(benchmark::kMillisecond);

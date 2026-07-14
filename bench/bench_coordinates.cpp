// Copyright (C) 2025 Kartik Vadhawana
//
// Coordinate-conversion throughput benchmarks (WGS84 geodetic -> ECEF -> ENU).

#include <benchmark/benchmark.h>

#include <vector>

#include "geometry/coordinates/wgs84.hpp"

using namespace map_matching_2::geometry::coordinates;

namespace {
    // A deterministic cloud of geodetic points around Karlsruhe.
    std::vector<GeodeticCoordinate> make_points(std::size_t n) {
        std::vector<GeodeticCoordinate> pts;
        pts.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const double t = static_cast<double>(i);
            pts.push_back({49.0 + 0.00001 * t, 8.4 + 0.00001 * t, 116.0 + 0.001 * t});
        }
        return pts;
    }
}

static void coord_geodetic_to_ecef(benchmark::State &state) {
    auto pts = make_points(static_cast<std::size_t>(state.range(0)));
    for (auto _ : state) {
        for (const auto &g : pts) {
            auto e = geodeticToEcef(g);
            benchmark::DoNotOptimize(e.x);
        }
    }
    state.SetItemsProcessed(state.iterations() * state.range(0));
}
BENCHMARK(coord_geodetic_to_ecef)->Arg(100000);

static void coord_geodetic_to_enu(benchmark::State &state) {
    auto pts = make_points(static_cast<std::size_t>(state.range(0)));
    EnuReferenceFrame frame{{49.0, 8.4, 116.0}};
    for (auto _ : state) {
        for (const auto &g : pts) {
            auto enu = frame.geodeticToEnu(g);
            benchmark::DoNotOptimize(enu.east);
        }
    }
    state.SetItemsProcessed(state.iterations() * state.range(0));
}
BENCHMARK(coord_geodetic_to_enu)->Arg(100000);

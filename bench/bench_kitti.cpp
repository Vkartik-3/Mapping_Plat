// Copyright (C) 2025 Kartik Vadhawana
//
// Ingestion / integrity benchmarks: OXTS parsing, Velodyne .bin reading,
// CRC32, ground extraction and frame validation.

#include <benchmark/benchmark.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "types/io/track/kitti_oxts_importer.hpp"
#include "types/io/lidar/kitti_lidar_reader.hpp"
#include "geometry/lidar/ground_extractor.hpp"
#include "geometry/lidar/frame_validator.hpp"
#include "util/crc32.hpp"
#include "util/synthetic_data.hpp"

using namespace map_matching_2;

namespace {

    // Write a synthetic OXTS sequence to a temp dir and return the path.
    std::filesystem::path make_oxts_sequence(int frames) {
        namespace fs = std::filesystem;
        fs::path dir = fs::temp_directory_path() / "hdmap_bench_oxts";
        fs::remove_all(dir);
        fs::create_directories(dir);
        auto traj = util::synthetic::trajectory(frames / 2, frames - frames / 2, 5.0);
        for (std::size_t i = 0; i < traj.size(); ++i) {
            char name[32];
            std::snprintf(name, sizeof(name), "%010zu.txt", i);
            std::ofstream out(dir / name);
            out << traj[i].lat << ' ' << traj[i].lon << " 116 0 0 1.5 5 0 5 0 0 0 0 9.8 0 0 "
                   "9.8 0 0 0.01 0.5 0.05 0 8 0 0 0 4 4 4\n";
        }
        return dir;
    }

    // Serialize a LiDAR frame to raw KITTI .bin bytes.
    std::vector<char> frame_to_bin(const io::lidar::LidarFrame &f) {
        std::vector<char> raw(f.points.size() * 16);
        std::memcpy(raw.data(), f.points.data(), raw.size());
        return raw;
    }

    // Compile-time default location of the real KITTI raw sequence on EC2.
#ifndef KITTI_DEFAULT_PATH
#define KITTI_DEFAULT_PATH \
    "/home/ubuntu/kitti/2011_09_26/2011_09_26/2011_09_26_drive_0001_sync/"
#endif

    // Root of the KITTI sequence: KITTI_PATH env var overrides the default.
    std::filesystem::path kitti_root() {
        if (const char *env = std::getenv("KITTI_PATH"); env && *env) {
            return std::filesystem::path(env);
        }
        return std::filesystem::path(KITTI_DEFAULT_PATH);
    }

    // Count *.<ext> files in dir, or 0 if the directory is absent.
    std::size_t count_files(const std::filesystem::path &dir, const std::string &ext) {
        namespace fs = std::filesystem;
        if (!fs::exists(dir) || !fs::is_directory(dir)) return 0;
        std::size_t n = 0;
        for (const auto &e : fs::directory_iterator(dir)) {
            if (e.is_regular_file() && e.path().extension() == ext) ++n;
        }
        return n;
    }

}

static void kitti_oxts_parse(benchmark::State &state) {
    auto dir = make_oxts_sequence(static_cast<int>(state.range(0)));
    for (auto _ : state) {
        auto seq = io::track::kitti_oxts_importer::load_sequence(dir);
        benchmark::DoNotOptimize(seq.frames_parsed);
    }
    state.SetItemsProcessed(state.iterations() * state.range(0));
}
BENCHMARK(kitti_oxts_parse)->Arg(200);

// Real KITTI OXTS: parse every frame in the sequence's oxts/data directory.
// Skips cleanly (no crash) when the dataset is not present on this machine.
static void kitti_oxts_parse_real(benchmark::State &state) {
    const std::filesystem::path dir = kitti_root() / "oxts" / "data";
    const std::size_t frames = count_files(dir, ".txt");
    if (frames == 0) {
        state.SkipWithError("[SKIP] Real KITTI benchmarks: path not found");
        return;
    }
    for (auto _ : state) {
        auto seq = io::track::kitti_oxts_importer::load_sequence(dir);
        benchmark::DoNotOptimize(seq.frames_parsed);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(frames));
}
BENCHMARK(kitti_oxts_parse_real);

static void kitti_lidar_read_1k(benchmark::State &state) {
    // Prepare 1000 small .bin frames on disk once.
    namespace fs = std::filesystem;
    fs::path dir = fs::temp_directory_path() / "hdmap_bench_velo";
    fs::remove_all(dir);
    fs::create_directories(dir);
    auto frame = util::synthetic::lidar_frame(1000, 200, -1.7f, 0, 11);
    auto raw = frame_to_bin(frame);
    for (int i = 0; i < 1000; ++i) {
        char name[32];
        std::snprintf(name, sizeof(name), "%010d.bin", i);
        std::ofstream(dir / name, std::ios::binary).write(raw.data(),
                static_cast<std::streamsize>(raw.size()));
    }
    for (auto _ : state) {
        io::lidar::LidarSequenceStats stats;
        auto frames = io::lidar::kitti_lidar_reader::read_sequence(dir, stats);
        benchmark::DoNotOptimize(stats.frames_read);
    }
    state.SetItemsProcessed(state.iterations() * 1000);
}
BENCHMARK(kitti_lidar_read_1k)->Unit(benchmark::kMillisecond);

// Real KITTI Velodyne: read every .bin frame in velodyne_points/data.
// Skips cleanly (no crash) when the dataset is not present on this machine.
static void kitti_lidar_read_real(benchmark::State &state) {
    const std::filesystem::path dir = kitti_root() / "velodyne_points" / "data";
    const std::size_t frames = count_files(dir, ".bin");
    if (frames == 0) {
        state.SkipWithError("[SKIP] Real KITTI benchmarks: path not found");
        return;
    }
    for (auto _ : state) {
        io::lidar::LidarSequenceStats stats;
        auto read = io::lidar::kitti_lidar_reader::read_sequence(dir, stats);
        benchmark::DoNotOptimize(stats.frames_read);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(frames));
}
BENCHMARK(kitti_lidar_read_real)->Unit(benchmark::kMillisecond);

static void crc32_1frame(benchmark::State &state) {
    auto frame = util::synthetic::lidar_frame(120000, 0, -1.7f, 0, 5);
    auto raw = frame_to_bin(frame);
    for (auto _ : state) {
        auto v = util::crc32::compute(raw.data(), raw.size());
        benchmark::DoNotOptimize(v);
    }
    state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(raw.size()));
}
BENCHMARK(crc32_1frame);

static void ground_extraction_1(benchmark::State &state) {
    auto frame = util::synthetic::lidar_frame(100000, 25000, -1.7f, 0, 5);
    geometry::lidar::ground_extractor ge;
    for (auto _ : state) {
        auto g = ge.extract(frame);
        benchmark::DoNotOptimize(g.ground_ratio);
    }
}
BENCHMARK(ground_extraction_1)->Unit(benchmark::kMillisecond);

static void frame_validate_100(benchmark::State &state) {
    auto frame = util::synthetic::lidar_frame(100000, 25000, -1.7f, 0, 5);
    geometry::lidar::frame_validator fv;
    for (auto _ : state) {
        for (int i = 0; i < 100; ++i) {
            auto r = fv.validate(frame, true);
            benchmark::DoNotOptimize(r.validation_pass);
        }
    }
    state.SetItemsProcessed(state.iterations() * 100);
}
BENCHMARK(frame_validate_100)->Unit(benchmark::kMillisecond);

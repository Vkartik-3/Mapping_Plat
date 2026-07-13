// Copyright (C) 2025 Kartik Vadhawana
//
// KITTI Velodyne point-cloud reader. Each frame is a raw little-endian
// float32 buffer of (x, y, z, intensity) tuples. Frames are CRC32-checked
// against an optional sidecar (.bin.crc) for integrity validation.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_IO_LIDAR_KITTI_LIDAR_READER_HPP
#define MAP_MATCHING_2_IO_LIDAR_KITTI_LIDAR_READER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <filesystem>

namespace map_matching_2::io::lidar {

    struct LidarPoint {
        float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
    };

    struct LidarFrame {
        std::uint64_t frame_index = 0;
        std::vector<LidarPoint> points;
        std::uint32_t crc32 = 0;        // computed over the raw .bin bytes
        bool crc_valid = false;         // true if it matched a sidecar value
        std::size_t point_count = 0;
    };

    // Aggregate statistics for reading a whole sequence directory.
    struct LidarSequenceStats {
        std::size_t frames_read = 0;
        std::size_t frames_failed = 0;
        std::size_t crc_mismatches = 0;
        double mean_points_per_frame = 0.0;
    };

    class kitti_lidar_reader {

    public:
        static constexpr std::size_t point_stride_bytes = 4 * sizeof(float); // 16
        static constexpr float coord_limit_m = 200.0f;

        // Read a single .bin frame. Computes CRC32 over the raw bytes and, if a
        // sidecar `<path>.crc` exists, verifies it (sets crc_valid). Throws
        // std::runtime_error if the file is missing or not a multiple of 16 B.
        static LidarFrame read_frame(const std::filesystem::path &bin_path,
                std::uint64_t frame_index = 0);

        // Read every *.bin in `velodyne_dir` (sorted by name) into frames,
        // filling `stats`. Frames that fail to read are counted, not thrown.
        static std::vector<LidarFrame> read_sequence(
                const std::filesystem::path &velodyne_dir, LidarSequenceStats &stats);

        // Write a `<bin_path>.crc` sidecar holding the frame's CRC32 as text,
        // so subsequent reads can verify integrity.
        static void write_crc_sidecar(const std::filesystem::path &bin_path,
                std::uint32_t crc);

        // Basic geometric sanity used during parsing: coords within
        // [-200, 200] m and intensity within [0, 1].
        static bool point_in_range(const LidarPoint &p);

    };

}

#endif //MAP_MATCHING_2_IO_LIDAR_KITTI_LIDAR_READER_HPP

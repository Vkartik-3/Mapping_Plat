// Copyright (C) 2025 Kartik Vadhawana
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include "types/io/lidar/kitti_lidar_reader.hpp"

#include "util/crc32.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace map_matching_2::io::lidar {

    bool kitti_lidar_reader::point_in_range(const LidarPoint &p) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                !std::isfinite(p.z) || !std::isfinite(p.intensity)) {
            return false;
        }
        if (std::fabs(p.x) > coord_limit_m || std::fabs(p.y) > coord_limit_m ||
                std::fabs(p.z) > coord_limit_m) {
            return false;
        }
        return p.intensity >= 0.0f && p.intensity <= 1.0f;
    }

    LidarFrame kitti_lidar_reader::read_frame(const std::filesystem::path &bin_path,
            std::uint64_t frame_index) {
        namespace fs = std::filesystem;
        if (!fs::exists(bin_path) || !fs::is_regular_file(bin_path)) {
            throw std::runtime_error("KITTI Velodyne file not found: " + bin_path.string());
        }

        std::ifstream in(bin_path, std::ios::binary);
        if (!in) {
            throw std::runtime_error("Cannot open KITTI Velodyne file: " + bin_path.string());
        }

        std::vector<char> raw((std::istreambuf_iterator<char>(in)),
                std::istreambuf_iterator<char>());

        if (raw.size() % point_stride_bytes != 0) {
            throw std::runtime_error(
                    "KITTI Velodyne file size is not a multiple of 16 bytes: " +
                    bin_path.string());
        }

        LidarFrame frame{};
        frame.frame_index = frame_index;
        frame.crc32 = util::crc32::compute(raw.data(), raw.size());

        const std::size_t count = raw.size() / point_stride_bytes;
        frame.points.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            LidarPoint p{};
            std::memcpy(&p, raw.data() + i * point_stride_bytes, point_stride_bytes);
            frame.points.push_back(p);
        }
        frame.point_count = frame.points.size();

        // Verify against a sidecar CRC if present.
        fs::path crc_path = bin_path;
        crc_path += ".crc";
        if (fs::exists(crc_path)) {
            std::ifstream crc_in(crc_path);
            std::uint32_t stored = 0;
            if (crc_in >> stored) {
                frame.crc_valid = (stored == frame.crc32);
            }
        }

        return frame;
    }

    void kitti_lidar_reader::write_crc_sidecar(const std::filesystem::path &bin_path,
            std::uint32_t crc) {
        std::filesystem::path crc_path = bin_path;
        crc_path += ".crc";
        std::ofstream out(crc_path);
        if (!out) {
            throw std::runtime_error("Cannot write CRC sidecar: " + crc_path.string());
        }
        out << crc << '\n';
    }

    std::vector<LidarFrame> kitti_lidar_reader::read_sequence(
            const std::filesystem::path &velodyne_dir, LidarSequenceStats &stats) {
        namespace fs = std::filesystem;
        stats = LidarSequenceStats{};

        if (!fs::exists(velodyne_dir) || !fs::is_directory(velodyne_dir)) {
            throw std::runtime_error(
                    "KITTI Velodyne directory not found: " + velodyne_dir.string());
        }

        std::vector<fs::path> files;
        for (const auto &entry : fs::directory_iterator(velodyne_dir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".bin") {
                files.push_back(entry.path());
            }
        }
        std::sort(files.begin(), files.end());

        std::vector<LidarFrame> frames;
        frames.reserve(files.size());
        std::size_t total_points = 0;
        std::uint64_t index = 0;
        for (const auto &file : files) {
            try {
                LidarFrame frame = read_frame(file, index);
                total_points += frame.point_count;
                if (fs::exists(fs::path(file).concat(".crc")) && !frame.crc_valid) {
                    ++stats.crc_mismatches;
                }
                frames.push_back(std::move(frame));
                ++stats.frames_read;
            } catch (const std::exception &) {
                ++stats.frames_failed;
            }
            ++index;
        }

        stats.mean_points_per_frame = stats.frames_read == 0
                ? 0.0
                : static_cast<double>(total_points) / static_cast<double>(stats.frames_read);

        return frames;
    }

}

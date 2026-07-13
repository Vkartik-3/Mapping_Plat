// Copyright (C) 2025 Kartik Vadhawana
//
// Synchronized KITTI raw sequence store. Loads an OXTS (GPS/IMU) directory
// and a Velodyne (.bin) directory and pairs them by frame index, exposing a
// unified per-frame view plus per-sequence quality statistics.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_IO_KITTI_SEQUENCE_HPP
#define MAP_MATCHING_2_IO_KITTI_SEQUENCE_HPP

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <filesystem>

#include "types/io/track/kitti_oxts_importer.hpp"
#include "types/io/lidar/kitti_lidar_reader.hpp"

namespace map_matching_2::io {

    // A single synchronized frame: one OXTS record and (optionally) one
    // LiDAR frame sharing the same index.
    struct KittiFrame {
        std::size_t index = 0;
        const track::KittiOxtsFrame *oxts = nullptr;
        const lidar::LidarFrame *lidar = nullptr;
    };

    struct KittiSequenceStats {
        std::size_t total_frames = 0;        // min(oxts, lidar) synchronized count
        std::size_t oxts_frames = 0;
        std::size_t lidar_frames = 0;
        std::size_t frames_with_gps_fix = 0; // navstat == 0 (KITTI: 0 == valid)
        std::size_t lidar_crc_failures = 0;
        double mean_points_per_frame = 0.0;
        // GPS trajectory bounding box in local ENU metres.
        double min_easting = 0.0, max_easting = 0.0;
        double min_northing = 0.0, max_northing = 0.0;
    };

    class KittiSequence {

    public:
        // Load an OXTS directory and a Velodyne directory. Either may be
        // empty; synchronization uses the overlapping index range.
        void load(const std::filesystem::path &oxts_dir,
                const std::filesystem::path &velodyne_dir) {
            _oxts = track::kitti_oxts_importer::load_sequence(oxts_dir);
            _lidar = lidar::kitti_lidar_reader::read_sequence(velodyne_dir, _lidar_stats);
            recompute_stats();
        }

        // Load only OXTS (GPS-only sequences / synthetic tests).
        void load_oxts(const std::filesystem::path &oxts_dir) {
            _oxts = track::kitti_oxts_importer::load_sequence(oxts_dir);
            recompute_stats();
        }

        [[nodiscard]] std::size_t frame_count() const noexcept {
            return std::max(_oxts.frames.size(), _lidar.size());
        }

        // Synchronized accessor: pairs OXTS and LiDAR by shared index.
        // Either pointer may be null if that modality lacks the index.
        [[nodiscard]] KittiFrame get_frame(std::size_t i) const {
            if (i >= frame_count()) {
                throw std::out_of_range("KittiSequence::get_frame index out of range");
            }
            KittiFrame frame{};
            frame.index = i;
            if (i < _oxts.frames.size()) frame.oxts = &_oxts.frames[i];
            if (i < _lidar.size()) frame.lidar = &_lidar[i];
            return frame;
        }

        [[nodiscard]] const track::KittiOxtsSequence &oxts() const noexcept { return _oxts; }
        [[nodiscard]] const std::vector<lidar::LidarFrame> &lidar() const noexcept { return _lidar; }
        [[nodiscard]] const KittiSequenceStats &stats() const noexcept { return _stats; }

    private:
        void recompute_stats() {
            _stats = KittiSequenceStats{};
            _stats.oxts_frames = _oxts.frames.size();
            _stats.lidar_frames = _lidar.size();
            _stats.total_frames = _lidar.empty()
                    ? _oxts.frames.size()
                    : std::min(_oxts.frames.size(), _lidar.size());
            _stats.lidar_crc_failures = _lidar_stats.crc_mismatches;
            _stats.mean_points_per_frame = _lidar_stats.mean_points_per_frame;

            for (const auto &f : _oxts.frames) {
                if (f.navstat == 0) ++_stats.frames_with_gps_fix;
            }

            if (!_oxts.frames.empty()) {
                double min_e = std::numeric_limits<double>::max();
                double max_e = std::numeric_limits<double>::lowest();
                double min_n = std::numeric_limits<double>::max();
                double max_n = std::numeric_limits<double>::lowest();
                for (const auto &f : _oxts.frames) {
                    min_e = std::min(min_e, f.easting);
                    max_e = std::max(max_e, f.easting);
                    min_n = std::min(min_n, f.northing);
                    max_n = std::max(max_n, f.northing);
                }
                _stats.min_easting = min_e;
                _stats.max_easting = max_e;
                _stats.min_northing = min_n;
                _stats.max_northing = max_n;
            }
        }

        track::KittiOxtsSequence _oxts{};
        std::vector<lidar::LidarFrame> _lidar{};
        lidar::LidarSequenceStats _lidar_stats{};
        KittiSequenceStats _stats{};
    };

}

#endif //MAP_MATCHING_2_IO_KITTI_SEQUENCE_HPP

// Copyright (C) 2025 Kartik Vadhawana
//
// KITTI raw OXTS (GPS/IMU) reader. Parses one .txt file per frame from a
// KITTI raw sequence and converts geodetic coordinates to a local ENU frame
// anchored at the first frame, for downstream trajectory / centerline mining.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_IO_TRACK_KITTI_OXTS_IMPORTER_HPP
#define MAP_MATCHING_2_IO_TRACK_KITTI_OXTS_IMPORTER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <filesystem>

namespace map_matching_2::io::track {

    // One parsed OXTS record. Field names follow the KITTI dataformat.txt.
    struct KittiOxtsFrame {
        std::uint64_t timestamp_ns = 0;    // derived from filename index if no timestamps file

        double lat = 0.0, lon = 0.0, alt = 0.0;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;

        double vn = 0.0, ve = 0.0, vf = 0.0;   // velocity north / east / forward
        double vl = 0.0, vu = 0.0;             // velocity leftward / upward

        double ax = 0.0, ay = 0.0, az = 0.0;   // acceleration (vehicle frame)
        double af = 0.0, al = 0.0, au = 0.0;   // acceleration forward / left / up
        double wx = 0.0, wy = 0.0, wz = 0.0;   // angular rate around x/y/z

        // Local ENU coordinates relative to the anchor (first) frame, metres.
        double easting = 0.0, northing = 0.0, up = 0.0;

        float pos_accuracy = 0.0f, vel_accuracy = 0.0f;
        int navstat = 0, numsats = 0;
        int posmode = 0, velmode = 0, orimode = 0;
    };

    // Result of importing a whole sequence directory. Malformed frames are
    // skipped and counted rather than aborting the whole load.
    struct KittiOxtsSequence {
        std::vector<KittiOxtsFrame> frames;
        double anchor_lat = 0.0, anchor_lon = 0.0, anchor_alt = 0.0;
        std::size_t frames_parsed = 0;
        std::size_t frames_skipped = 0;
    };

    class kitti_oxts_importer {

    public:
        // Radius used for the Haversine great-circle projection (WGS84 mean).
        static constexpr double earth_radius_m = 6371000.0;

        // Parse a single OXTS record from one whitespace-separated line.
        // Returns false if the line is malformed or holds out-of-range values.
        static bool parse_line(const std::string &line, KittiOxtsFrame &out);

        // Signed great-circle distance components (Haversine) from the anchor
        // to (lat, lon), giving a local east/north tangent-plane offset.
        static void to_enu(double anchor_lat, double anchor_lon, double anchor_alt,
                double lat, double lon, double alt,
                double &easting, double &northing, double &up);

        // Load every *.txt in `oxts_data_dir` (sorted by name), anchoring ENU
        // to the first successfully parsed frame. Throws std::runtime_error if
        // the directory does not exist.
        static KittiOxtsSequence load_sequence(const std::filesystem::path &oxts_data_dir);

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_KITTI_OXTS_IMPORTER_HPP

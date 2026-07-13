// Copyright (C) 2025 Kartik Vadhawana
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include "types/io/track/kitti_oxts_importer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace map_matching_2::io::track {

    bool kitti_oxts_importer::parse_line(const std::string &line, KittiOxtsFrame &out) {
        std::istringstream iss(line);
        std::vector<double> v;
        v.reserve(30);
        double value;
        while (iss >> value) {
            if (!std::isfinite(value)) {
                return false;
            }
            v.push_back(value);
        }

        // KITTI OXTS records carry 30 fields. Accept the first 25 numeric
        // fields as the minimum core set so slightly truncated dumps still load.
        if (v.size() < 25) {
            return false;
        }

        KittiOxtsFrame f{};
        f.lat = v[0];
        f.lon = v[1];
        f.alt = v[2];
        f.roll = v[3];
        f.pitch = v[4];
        f.yaw = v[5];
        f.vn = v[6];
        f.ve = v[7];
        f.vf = v[8];
        f.vl = v[9];
        f.vu = v[10];
        f.ax = v[11];
        f.ay = v[12];
        f.az = v[13];
        f.af = v[14];
        f.al = v[15];
        f.au = v[16];
        f.wx = v[17];
        f.wy = v[18];
        f.wz = v[19];
        f.pos_accuracy = static_cast<float>(v[20]);
        f.vel_accuracy = static_cast<float>(v[21]);
        f.navstat = static_cast<int>(v[22]);
        f.numsats = static_cast<int>(v[23]);
        f.posmode = static_cast<int>(v[24]);
        if (v.size() > 25) f.velmode = static_cast<int>(v[25]);
        if (v.size() > 26) f.orimode = static_cast<int>(v[26]);

        // Range checks on the geodetic fields.
        if (f.lat < -90.0 || f.lat > 90.0 || f.lon < -180.0 || f.lon > 180.0) {
            return false;
        }

        out = f;
        return true;
    }

    void kitti_oxts_importer::to_enu(double anchor_lat, double anchor_lon, double anchor_alt,
            double lat, double lon, double alt,
            double &easting, double &northing, double &up) {
        constexpr double deg2rad = M_PI / 180.0;

        const auto haversine = [](double lat1, double lon1, double lat2, double lon2) {
            const double dlat = (lat2 - lat1) * deg2rad;
            const double dlon = (lon2 - lon1) * deg2rad;
            const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                    std::cos(lat1 * deg2rad) * std::cos(lat2 * deg2rad) *
                            std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
            const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
            return earth_radius_m * c;
        };

        // North component: vary latitude only, hold longitude at anchor.
        northing = haversine(anchor_lat, anchor_lon, lat, anchor_lon);
        if (lat < anchor_lat) northing = -northing;

        // East component: vary longitude only, hold latitude at anchor.
        easting = haversine(anchor_lat, anchor_lon, anchor_lat, lon);
        if (lon < anchor_lon) easting = -easting;

        up = alt - anchor_alt;
    }

    KittiOxtsSequence kitti_oxts_importer::load_sequence(
            const std::filesystem::path &oxts_data_dir) {
        namespace fs = std::filesystem;
        if (!fs::exists(oxts_data_dir) || !fs::is_directory(oxts_data_dir)) {
            throw std::runtime_error(
                    "KITTI OXTS directory not found: " + oxts_data_dir.string());
        }

        std::vector<fs::path> files;
        for (const auto &entry : fs::directory_iterator(oxts_data_dir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".txt") {
                files.push_back(entry.path());
            }
        }
        std::sort(files.begin(), files.end());

        KittiOxtsSequence seq{};
        bool anchor_set = false;
        std::uint64_t index = 0;
        for (const auto &file : files) {
            std::ifstream in(file);
            if (!in) {
                ++seq.frames_skipped;
                ++index;
                continue;
            }
            std::string line;
            std::getline(in, line);

            KittiOxtsFrame frame{};
            if (!parse_line(line, frame)) {
                ++seq.frames_skipped;
                ++index;
                continue;
            }

            // Timestamp derived from filename index (0000000000.txt -> index).
            // 10 Hz nominal sample rate for KITTI raw OXTS.
            frame.timestamp_ns = index * 100'000'000ULL;

            if (!anchor_set) {
                seq.anchor_lat = frame.lat;
                seq.anchor_lon = frame.lon;
                seq.anchor_alt = frame.alt;
                anchor_set = true;
            }
            to_enu(seq.anchor_lat, seq.anchor_lon, seq.anchor_alt,
                    frame.lat, frame.lon, frame.alt,
                    frame.easting, frame.northing, frame.up);

            seq.frames.push_back(frame);
            ++seq.frames_parsed;
            ++index;
        }

        return seq;
    }

}

// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "types/io/track/kitti_oxts_importer.hpp"
#include "types/io/lidar/kitti_lidar_reader.hpp"
#include "util/crc32.hpp"

namespace fs = std::filesystem;
using namespace map_matching_2::io;

namespace {
    fs::path make_temp_dir(const std::string &name) {
        fs::path dir = fs::temp_directory_path() / ("hdmap_test_" + name);
        fs::remove_all(dir);
        fs::create_directories(dir);
        return dir;
    }
}

TEST(KittiOxts, ParsesValidFrame) {
    track::KittiOxtsFrame f{};
    const std::string line =
            "49.0 8.4 116.0 0.01 0.02 1.5 5.0 0.1 5.0 0.0 0.0 0.1 0.0 9.8 0.1 0.0 "
            "9.8 0.0 0.0 0.01 0.5 0.05 0 8 0 0 0 4 4 4";
    ASSERT_TRUE(track::kitti_oxts_importer::parse_line(line, f));
    EXPECT_DOUBLE_EQ(f.lat, 49.0);
    EXPECT_DOUBLE_EQ(f.lon, 8.4);
    EXPECT_EQ(f.numsats, 8);
    EXPECT_EQ(f.navstat, 0);
}

TEST(KittiOxts, RejectsMalformedLine) {
    track::KittiOxtsFrame f{};
    EXPECT_FALSE(track::kitti_oxts_importer::parse_line("garbage not a frame", f));
}

TEST(KittiOxts, RejectsOutOfRangeLatLon) {
    track::KittiOxtsFrame f{};
    std::string line = "500.0 8.4 116 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0";
    EXPECT_FALSE(track::kitti_oxts_importer::parse_line(line, f));
}

TEST(KittiOxts, EnuConversionAccuracy) {
    // ~0.00009 deg latitude north of the anchor should be ~10 m north.
    double e, n, u;
    track::kitti_oxts_importer::to_enu(49.0, 8.4, 116.0, 49.00009, 8.4, 116.0, e, n, u);
    EXPECT_NEAR(e, 0.0, 1e-6);
    EXPECT_NEAR(n, 10.0, 0.05);
    EXPECT_NEAR(u, 0.0, 1e-9);
}

TEST(KittiOxts, LoadSequenceSkipsBadFramesAndAnchors) {
    fs::path dir = make_temp_dir("oxts");
    std::ofstream(dir / "0000000000.txt")
            << "49.0 8.4 116 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0\n";
    std::ofstream(dir / "0000000001.txt")
            << "49.00009 8.4 116 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0\n";
    std::ofstream(dir / "0000000002.txt") << "broken\n";
    auto seq = track::kitti_oxts_importer::load_sequence(dir);
    EXPECT_EQ(seq.frames_parsed, 2u);
    EXPECT_EQ(seq.frames_skipped, 1u);
    EXPECT_DOUBLE_EQ(seq.anchor_lat, 49.0);
    EXPECT_NEAR(seq.frames[1].northing, 10.0, 0.05);
}

TEST(KittiOxts, MissingDirectoryThrows) {
    EXPECT_THROW(track::kitti_oxts_importer::load_sequence("/no/such/dir/xyz"),
            std::runtime_error);
}

namespace {
    void write_bin(const fs::path &p, const std::vector<float> &vals) {
        std::ofstream out(p, std::ios::binary);
        out.write(reinterpret_cast<const char *>(vals.data()),
                static_cast<std::streamsize>(vals.size() * sizeof(float)));
    }
}

TEST(KittiLidar, ReadsValidBin) {
    fs::path dir = make_temp_dir("velo");
    fs::path bin = dir / "0000000000.bin";
    std::vector<float> vals;
    for (int i = 0; i < 100; ++i) { vals.push_back(i); vals.push_back(-i); vals.push_back(0); vals.push_back(0.5f); }
    write_bin(bin, vals);
    auto frame = lidar::kitti_lidar_reader::read_frame(bin, 0);
    EXPECT_EQ(frame.point_count, 100u);
    EXPECT_FLOAT_EQ(frame.points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(frame.points[1].intensity, 0.5f);
}

TEST(KittiLidar, WrongSizeThrows) {
    fs::path dir = make_temp_dir("velo_badsize");
    fs::path bin = dir / "bad.bin";
    std::ofstream out(bin, std::ios::binary);
    const char junk[13] = "not16aligned"; // 13 bytes, not multiple of 16
    out.write(junk, 13);
    out.close();
    EXPECT_THROW(lidar::kitti_lidar_reader::read_frame(bin, 0), std::runtime_error);
}

TEST(KittiLidar, CrcSidecarRoundTripDetectsMismatch) {
    fs::path dir = make_temp_dir("velo_crc");
    fs::path bin = dir / "0000000000.bin";
    write_bin(bin, {1, 2, 3, 0.5f, 4, 5, 6, 0.7f});
    auto frame = lidar::kitti_lidar_reader::read_frame(bin, 0);
    lidar::kitti_lidar_reader::write_crc_sidecar(bin, frame.crc32);
    auto ok = lidar::kitti_lidar_reader::read_frame(bin, 0);
    EXPECT_TRUE(ok.crc_valid);
    // corrupt the sidecar
    std::ofstream(dir / "0000000000.bin.crc") << (frame.crc32 ^ 0x1u) << "\n";
    auto bad = lidar::kitti_lidar_reader::read_frame(bin, 0);
    EXPECT_FALSE(bad.crc_valid);
}

TEST(KittiLidar, PointInRangeDetectsNaN) {
    lidar::LidarPoint nan_pt{std::nanf(""), 0, 0, 0.5f};
    lidar::LidarPoint ok_pt{1, 2, 3, 0.5f};
    lidar::LidarPoint far_pt{1000, 0, 0, 0.5f};
    EXPECT_FALSE(lidar::kitti_lidar_reader::point_in_range(nan_pt));
    EXPECT_TRUE(lidar::kitti_lidar_reader::point_in_range(ok_pt));
    EXPECT_FALSE(lidar::kitti_lidar_reader::point_in_range(far_pt));
}

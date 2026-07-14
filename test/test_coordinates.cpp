// Copyright (C) 2025 Kartik Vadhawana
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "geometry/coordinates/wgs84.hpp"

using namespace map_matching_2::geometry::coordinates;

// --- Trusted reference ECEF values (WGS84) ---------------------------------
// (lat, lon, alt) = (0, 0, 0)  ->  (a, 0, 0)
TEST(Wgs84Ecef, EquatorPrimeMeridian) {
    auto e = geodeticToEcef({0.0, 0.0, 0.0});
    EXPECT_NEAR(e.x, Wgs84::semi_major_axis, 1e-3);
    EXPECT_NEAR(e.y, 0.0, 1e-6);
    EXPECT_NEAR(e.z, 0.0, 1e-6);
}

// North pole -> (0, 0, b) where b = a(1-f).
TEST(Wgs84Ecef, NorthPole) {
    auto e = geodeticToEcef({90.0, 0.0, 0.0});
    EXPECT_NEAR(e.x, 0.0, 1e-3);
    EXPECT_NEAR(e.y, 0.0, 1e-3);
    EXPECT_NEAR(e.z, Wgs84::semi_minor_axis, 1e-3);
}

// (0, 90, 0) -> (0, a, 0).
TEST(Wgs84Ecef, EquatorNinetyEast) {
    auto e = geodeticToEcef({0.0, 90.0, 0.0});
    EXPECT_NEAR(e.x, 0.0, 1e-3);
    EXPECT_NEAR(e.y, Wgs84::semi_major_axis, 1e-3);
    EXPECT_NEAR(e.z, 0.0, 1e-6);
}

TEST(Wgs84Ecef, RoundTripGeodeticEcef) {
    GeodeticCoordinate g{49.006, 8.401, 116.4};
    auto back = ecefToGeodetic(geodeticToEcef(g));
    EXPECT_NEAR(back.latitude_deg, g.latitude_deg, 1e-8);
    EXPECT_NEAR(back.longitude_deg, g.longitude_deg, 1e-8);
    EXPECT_NEAR(back.altitude_m, g.altitude_m, 1e-4);
}

// --- ENU frame behaviour ----------------------------------------------------
TEST(Enu, AnchorMapsToOrigin) {
    GeodeticCoordinate anchor{49.0, 8.4, 116.0};
    EnuReferenceFrame frame{anchor};
    auto enu = frame.geodeticToEnu(anchor);
    EXPECT_NEAR(enu.east, 0.0, 1e-6);
    EXPECT_NEAR(enu.north, 0.0, 1e-6);
    EXPECT_NEAR(enu.up, 0.0, 1e-6);
}

TEST(Enu, EastwardPointHasPositiveEast) {
    EnuReferenceFrame frame{{49.0, 8.4, 116.0}};
    auto enu = frame.geodeticToEnu({49.0, 8.4010, 116.0}); // +longitude => east
    EXPECT_GT(enu.east, 0.0);
    EXPECT_NEAR(enu.north, 0.0, 0.05);
    EXPECT_NEAR(enu.up, 0.0, 0.05);
}

TEST(Enu, NorthwardPointHasPositiveNorth) {
    EnuReferenceFrame frame{{49.0, 8.4, 116.0}};
    auto enu = frame.geodeticToEnu({49.0009, 8.4, 116.0}); // +latitude => north
    EXPECT_GT(enu.north, 0.0);
    EXPECT_NEAR(enu.east, 0.0, 0.05);
    // ~100 m north at this latitude.
    EXPECT_NEAR(enu.north, 100.0, 1.0);
}

TEST(Enu, HigherAltitudeHasPositiveUp) {
    EnuReferenceFrame frame{{49.0, 8.4, 116.0}};
    auto enu = frame.geodeticToEnu({49.0, 8.4, 126.0}); // +10 m altitude
    EXPECT_NEAR(enu.up, 10.0, 1e-3);
    EXPECT_NEAR(enu.east, 0.0, 1e-3);
    EXPECT_NEAR(enu.north, 0.0, 1e-3);
}

TEST(Enu, RoundTripEnuGeodetic) {
    EnuReferenceFrame frame{{49.0, 8.4, 116.0}};
    GeodeticCoordinate g{49.0031, 8.4057, 121.0};
    auto enu = frame.geodeticToEnu(g);
    auto back = frame.enuToGeodetic(enu);
    EXPECT_NEAR(back.latitude_deg, g.latitude_deg, 1e-8);
    EXPECT_NEAR(back.longitude_deg, g.longitude_deg, 1e-8);
    EXPECT_NEAR(back.altitude_m, g.altitude_m, 1e-3);
}

// A short east+north displacement should have ENU distance ~ the straight-line
// ground distance (documented tolerance: sub-metre over ~140 m).
TEST(Enu, KnownDisplacementMagnitude) {
    EnuReferenceFrame frame{{49.0, 8.4, 116.0}};
    auto enu = frame.geodeticToEnu({49.0009, 8.4010, 116.0});
    const double horiz = std::sqrt(enu.east * enu.east + enu.north * enu.north);
    // ~100 m north + ~73 m east => ~124 m.
    EXPECT_GT(horiz, 115.0);
    EXPECT_LT(horiz, 135.0);
}

// --- Validation -------------------------------------------------------------
TEST(Wgs84Validation, RejectsOutOfRangeLatitude) {
    EXPECT_FALSE(is_valid({95.0, 8.4, 0.0}));
    auto convert = [] { auto e = geodeticToEcef({95.0, 8.4, 0.0}); return e.x; };
    EXPECT_THROW(convert(), std::invalid_argument);
}

TEST(Wgs84Validation, RejectsOutOfRangeLongitude) {
    EXPECT_FALSE(is_valid({49.0, 200.0, 0.0}));
    auto convert = [] { auto e = geodeticToEcef({49.0, 200.0, 0.0}); return e.x; };
    EXPECT_THROW(convert(), std::invalid_argument);
}

TEST(Wgs84Validation, InvalidAnchorThrows) {
    auto make = [] { return EnuReferenceFrame{GeodeticCoordinate{-91.0, 0.0, 0.0}}; };
    EXPECT_THROW(make(), std::invalid_argument);
}

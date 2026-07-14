// Copyright (C) 2025 Kartik Vadhawana
//
// True WGS84 geodetic <-> ECEF <-> local ENU coordinate transforms.
//
// This replaces an earlier approximate "ENU" that used signed Haversine /
// equirectangular offsets. Here the transform is the standard, mathematically
// correct chain:
//
//     geodetic (lat, lon, alt)  ->  ECEF (x, y, z)  ->  local ENU (E, N, U)
//
// around a fixed anchor (EnuReferenceFrame). All computation is double
// precision and altitude is preserved throughout. KITTI GPS points and OSM
// road nodes share one EnuReferenceFrame so they live in the exact same metric
// frame.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_COORDINATES_WGS84_HPP
#define MAP_MATCHING_2_GEOMETRY_COORDINATES_WGS84_HPP

namespace map_matching_2::geometry::coordinates {

    // WGS84 ellipsoid constants.
    struct Wgs84 {
        static constexpr double semi_major_axis = 6378137.0;                 // a, metres
        static constexpr double flattening = 1.0 / 298.257223563;            // f
        static constexpr double semi_minor_axis =
                semi_major_axis * (1.0 - flattening);                        // b, metres
        // First eccentricity squared: e^2 = f (2 - f).
        static constexpr double first_eccentricity_sq = flattening * (2.0 - flattening);
    };

    struct GeodeticCoordinate {
        double latitude_deg = 0.0;   // [-90, 90]
        double longitude_deg = 0.0;  // [-180, 180]
        double altitude_m = 0.0;     // metres above the ellipsoid
    };

    struct EcefCoordinate {
        double x = 0.0, y = 0.0, z = 0.0; // metres, Earth-Centered Earth-Fixed
    };

    struct EnuCoordinate {
        double east = 0.0, north = 0.0, up = 0.0; // metres, local tangent plane
    };

    // Returns true if a geodetic coordinate is within valid latitude/longitude
    // ranges and finite.
    [[nodiscard]] bool is_valid(const GeodeticCoordinate &g) noexcept;

    // geodetic -> ECEF (WGS84). Throws std::invalid_argument if `g` is invalid.
    [[nodiscard]] EcefCoordinate geodeticToEcef(const GeodeticCoordinate &g);

    // ECEF -> geodetic (WGS84), closed-form Bowring iteration. Used for the
    // inverse path (e.g. ENU results back to lat/lon for visualization).
    [[nodiscard]] GeodeticCoordinate ecefToGeodetic(const EcefCoordinate &e) noexcept;

    // Fixed anchor for a local ENU tangent frame. Precomputes the anchor's ECEF
    // position and the rotation trig so per-point conversion is cheap.
    class EnuReferenceFrame {
    public:
        EnuReferenceFrame() = default;

        // Anchor the frame at a geodetic origin (e.g. the first KITTI OXTS
        // frame). Throws std::invalid_argument if the anchor is invalid.
        explicit EnuReferenceFrame(const GeodeticCoordinate &anchor);

        [[nodiscard]] const GeodeticCoordinate &anchor() const noexcept { return _anchor; }
        [[nodiscard]] const EcefCoordinate &anchor_ecef() const noexcept { return _anchor_ecef; }

        // ECEF -> ENU about this anchor.
        [[nodiscard]] EnuCoordinate ecefToEnu(const EcefCoordinate &e) const noexcept;

        // geodetic -> ENU about this anchor (throws on invalid input).
        [[nodiscard]] EnuCoordinate geodeticToEnu(const GeodeticCoordinate &g) const;

        // Inverse: ENU -> ECEF -> geodetic about this anchor.
        [[nodiscard]] EcefCoordinate enuToEcef(const EnuCoordinate &enu) const noexcept;
        [[nodiscard]] GeodeticCoordinate enuToGeodetic(const EnuCoordinate &enu) const noexcept;

    private:
        GeodeticCoordinate _anchor{};
        EcefCoordinate _anchor_ecef{};
        double _sin_lat = 0.0, _cos_lat = 1.0;
        double _sin_lon = 0.0, _cos_lon = 1.0;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_COORDINATES_WGS84_HPP

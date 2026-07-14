// Copyright (C) 2025 Kartik Vadhawana
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include "geometry/coordinates/wgs84.hpp"

#include <cmath>
#include <stdexcept>

namespace map_matching_2::geometry::coordinates {

    namespace {
        constexpr double deg2rad = 3.14159265358979323846 / 180.0;
    }

    bool is_valid(const GeodeticCoordinate &g) noexcept {
        return std::isfinite(g.latitude_deg) && std::isfinite(g.longitude_deg) &&
                std::isfinite(g.altitude_m) &&
                g.latitude_deg >= -90.0 && g.latitude_deg <= 90.0 &&
                g.longitude_deg >= -180.0 && g.longitude_deg <= 180.0;
    }

    EcefCoordinate geodeticToEcef(const GeodeticCoordinate &g) {
        if (!is_valid(g)) {
            throw std::invalid_argument("geodeticToEcef: latitude/longitude out of range");
        }
        const double lat = g.latitude_deg * deg2rad;
        const double lon = g.longitude_deg * deg2rad;
        const double sin_lat = std::sin(lat);
        const double cos_lat = std::cos(lat);
        const double sin_lon = std::sin(lon);
        const double cos_lon = std::cos(lon);

        // Prime vertical radius of curvature.
        const double N = Wgs84::semi_major_axis /
                std::sqrt(1.0 - Wgs84::first_eccentricity_sq * sin_lat * sin_lat);

        EcefCoordinate e{};
        e.x = (N + g.altitude_m) * cos_lat * cos_lon;
        e.y = (N + g.altitude_m) * cos_lat * sin_lon;
        e.z = (N * (1.0 - Wgs84::first_eccentricity_sq) + g.altitude_m) * sin_lat;
        return e;
    }

    GeodeticCoordinate ecefToGeodetic(const EcefCoordinate &e) noexcept {
        // Closed-form Bowring method.
        const double a = Wgs84::semi_major_axis;
        const double b = Wgs84::semi_minor_axis;
        const double e2 = Wgs84::first_eccentricity_sq;
        const double ep2 = (a * a - b * b) / (b * b); // second eccentricity squared

        const double p = std::sqrt(e.x * e.x + e.y * e.y);
        GeodeticCoordinate g{};
        g.longitude_deg = std::atan2(e.y, e.x) / deg2rad;

        if (p < 1e-9) {
            // Pole.
            g.latitude_deg = (e.z >= 0.0 ? 90.0 : -90.0);
            g.altitude_m = std::fabs(e.z) - b;
            return g;
        }

        const double theta = std::atan2(e.z * a, p * b);
        const double sin_t = std::sin(theta);
        const double cos_t = std::cos(theta);
        const double lat = std::atan2(e.z + ep2 * b * sin_t * sin_t * sin_t,
                p - e2 * a * cos_t * cos_t * cos_t);
        const double sin_lat = std::sin(lat);
        const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

        g.latitude_deg = lat / deg2rad;
        g.altitude_m = p / std::cos(lat) - N;
        return g;
    }

    EnuReferenceFrame::EnuReferenceFrame(const GeodeticCoordinate &anchor)
        : _anchor(anchor) {
        if (!is_valid(anchor)) {
            throw std::invalid_argument("EnuReferenceFrame: invalid anchor");
        }
        _anchor_ecef = geodeticToEcef(anchor);
        const double lat = anchor.latitude_deg * deg2rad;
        const double lon = anchor.longitude_deg * deg2rad;
        _sin_lat = std::sin(lat);
        _cos_lat = std::cos(lat);
        _sin_lon = std::sin(lon);
        _cos_lon = std::cos(lon);
    }

    EnuCoordinate EnuReferenceFrame::ecefToEnu(const EcefCoordinate &e) const noexcept {
        const double dx = e.x - _anchor_ecef.x;
        const double dy = e.y - _anchor_ecef.y;
        const double dz = e.z - _anchor_ecef.z;

        EnuCoordinate enu{};
        enu.east = -_sin_lon * dx + _cos_lon * dy;
        enu.north = -_sin_lat * _cos_lon * dx - _sin_lat * _sin_lon * dy + _cos_lat * dz;
        enu.up = _cos_lat * _cos_lon * dx + _cos_lat * _sin_lon * dy + _sin_lat * dz;
        return enu;
    }

    EnuCoordinate EnuReferenceFrame::geodeticToEnu(const GeodeticCoordinate &g) const {
        return ecefToEnu(geodeticToEcef(g));
    }

    EcefCoordinate EnuReferenceFrame::enuToEcef(const EnuCoordinate &enu) const noexcept {
        // Transpose of the ecefToEnu rotation, translated back to the anchor.
        const double dx = -_sin_lon * enu.east - _sin_lat * _cos_lon * enu.north +
                _cos_lat * _cos_lon * enu.up;
        const double dy = _cos_lon * enu.east - _sin_lat * _sin_lon * enu.north +
                _cos_lat * _sin_lon * enu.up;
        const double dz = _cos_lat * enu.north + _sin_lat * enu.up;
        EcefCoordinate e{};
        e.x = _anchor_ecef.x + dx;
        e.y = _anchor_ecef.y + dy;
        e.z = _anchor_ecef.z + dz;
        return e;
    }

    GeodeticCoordinate EnuReferenceFrame::enuToGeodetic(const EnuCoordinate &enu) const noexcept {
        return ecefToGeodetic(enuToEcef(enu));
    }

}

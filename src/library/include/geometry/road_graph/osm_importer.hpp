// Copyright (C) 2025 Kartik Vadhawana
//
// Self-contained OSM (.osm XML) road-network importer. Parses <node> and
// <way> elements, keeps highway=* ways of the configured classes, and builds
// a RoadGraph in a local ENU frame anchored to a caller-supplied origin
// (typically the KITTI sequence anchor, so GPS and OSM share coordinates).
//
// This is a lightweight, dependency-free path used by the HD map pipeline;
// the upstream libosmium importer remains available for production PBF loads.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#ifndef MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_OSM_IMPORTER_HPP
#define MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_OSM_IMPORTER_HPP

#include <filesystem>
#include <string>
#include <vector>

#include "geometry/road_graph/road_graph.hpp"
#include "geometry/coordinates/wgs84.hpp"

namespace map_matching_2::geometry::road_graph {

    struct OsmImportParams {
        // ENU anchor. Use the KITTI OXTS sequence anchor so GPS and OSM share
        // the exact same true-WGS84 ENU frame.
        double anchor_lat = 0.0;
        double anchor_lon = 0.0;
        double anchor_alt = 0.0;
        // highway=* values to keep. Empty => keep a sensible driving default.
        std::vector<std::string> keep_classes = {
                "motorway", "trunk", "primary", "secondary", "tertiary",
                "residential", "unclassified", "living_street", "service"};
    };

    class osm_importer {

    public:
        explicit osm_importer(OsmImportParams params)
            : _params(std::move(params)),
              _frame(coordinates::GeodeticCoordinate{
                      _params.anchor_lat, _params.anchor_lon, _params.anchor_alt}) {}

        // Parse an .osm XML file into a RoadGraph. Throws std::runtime_error if
        // the file cannot be opened.
        [[nodiscard]] RoadGraph import_file(const std::filesystem::path &osm_path) const;

        // Default speed limit (km/h) by highway class when maxspeed is absent.
        static double default_speed(const std::string &road_class);

        // Binary serialization of a RoadGraph (uses the project's local format,
        // not the mmapped storage). Round-trips with load_binary.
        static void save_binary(const RoadGraph &graph, const std::filesystem::path &out);
        [[nodiscard]] static RoadGraph load_binary(const std::filesystem::path &in);

    private:
        void to_enu(double lat, double lon, double &e, double &n) const;

        OsmImportParams _params;
        coordinates::EnuReferenceFrame _frame;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_ROAD_GRAPH_OSM_IMPORTER_HPP

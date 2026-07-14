// Copyright (C) 2025 Kartik Vadhawana
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include "geometry/road_graph/osm_importer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace map_matching_2::geometry::road_graph {

    namespace {

        // Extract the value of attribute `attr` from an XML element string.
        // Returns empty if not present. Minimal, tolerant of single/double quotes.
        std::string attr_value(const std::string &element, const std::string &attr) {
            const std::string key = attr + "=";
            std::size_t p = element.find(key);
            if (p == std::string::npos) return {};
            p += key.size();
            if (p >= element.size()) return {};
            const char q = element[p];
            if (q != '"' && q != '\'') return {};
            const std::size_t end = element.find(q, p + 1);
            if (end == std::string::npos) return {};
            return element.substr(p + 1, end - p - 1);
        }

    }

    double osm_importer::default_speed(const std::string &road_class) {
        if (road_class == "motorway") return 120.0;
        if (road_class == "trunk") return 100.0;
        if (road_class == "primary") return 80.0;
        if (road_class == "secondary") return 60.0;
        if (road_class == "tertiary") return 50.0;
        if (road_class == "residential" || road_class == "unclassified") return 50.0;
        if (road_class == "living_street") return 20.0;
        if (road_class == "service") return 30.0;
        return 50.0;
    }

    void osm_importer::to_enu(double lat, double lon, double &e, double &n) const {
        // True WGS84 ENU about the shared anchor. OSM nodes carry no altitude,
        // so they are placed at the anchor altitude -> up ~ 0, keeping the road
        // graph on the anchor's local tangent plane.
        const geometry::coordinates::EnuCoordinate enu = _frame.geodeticToEnu(
                geometry::coordinates::GeodeticCoordinate{lat, lon, _params.anchor_alt});
        e = enu.east;
        n = enu.north;
    }

    RoadGraph osm_importer::import_file(const std::filesystem::path &osm_path) const {
        std::ifstream in(osm_path);
        if (!in) {
            throw std::runtime_error("Cannot open OSM file: " + osm_path.string());
        }

        struct RawNode { double lat, lon; };
        std::unordered_map<std::uint64_t, RawNode> raw_nodes;

        struct RawWay {
            std::vector<std::uint64_t> refs;
            std::string highway;
            double maxspeed = 0.0;
        };
        std::vector<RawWay> raw_ways;

        std::string line;
        RawWay *current_way = nullptr;
        bool in_way = false;
        RawWay building{};

        while (std::getline(in, line)) {
            // Trim leading whitespace for prefix checks.
            std::size_t s = line.find_first_not_of(" \t");
            if (s == std::string::npos) continue;
            std::string_view sv(line);
            sv.remove_prefix(s);

            if (sv.rfind("<node", 0) == 0) {
                const std::string id = attr_value(line, "id");
                const std::string lat = attr_value(line, "lat");
                const std::string lon = attr_value(line, "lon");
                if (!id.empty() && !lat.empty() && !lon.empty()) {
                    raw_nodes[std::stoull(id)] = {std::stod(lat), std::stod(lon)};
                }
            } else if (sv.rfind("<way", 0) == 0) {
                in_way = true;
                building = RawWay{};
                current_way = &building;
            } else if (in_way && sv.rfind("<nd", 0) == 0) {
                const std::string ref = attr_value(line, "ref");
                if (!ref.empty()) current_way->refs.push_back(std::stoull(ref));
            } else if (in_way && sv.rfind("<tag", 0) == 0) {
                const std::string k = attr_value(line, "k");
                const std::string v = attr_value(line, "v");
                if (k == "highway") current_way->highway = v;
                else if (k == "maxspeed") {
                    try { current_way->maxspeed = std::stod(v); } catch (...) {}
                }
            } else if (in_way && sv.rfind("</way", 0) == 0) {
                if (!current_way->highway.empty()) raw_ways.push_back(*current_way);
                in_way = false;
                current_way = nullptr;
            }
        }

        // Build the graph: keep ways of the configured classes, materialize
        // their referenced nodes, and split each way into consecutive edges.
        RoadGraph graph;
        std::unordered_map<std::uint64_t, std::size_t> node_index;

        const auto want_class = [&](const std::string &hw) {
            return std::find(_params.keep_classes.begin(), _params.keep_classes.end(), hw)
                    != _params.keep_classes.end();
        };
        const auto get_node = [&](std::uint64_t osm_id) -> long long {
            auto it = raw_nodes.find(osm_id);
            if (it == raw_nodes.end()) return -1;
            auto found = node_index.find(osm_id);
            if (found != node_index.end()) return static_cast<long long>(found->second);
            RoadNode node{};
            node.id = osm_id;
            node.lat = it->second.lat;
            node.lon = it->second.lon;
            to_enu(node.lat, node.lon, node.pos.e, node.pos.n);
            const std::size_t idx = graph.nodes.size();
            graph.nodes.push_back(node);
            node_index[osm_id] = idx;
            return static_cast<long long>(idx);
        };

        std::uint64_t edge_id = 0;
        for (const auto &way : raw_ways) {
            if (!want_class(way.highway) || way.refs.size() < 2) continue;
            const double speed = way.maxspeed > 0.0 ? way.maxspeed : default_speed(way.highway);
            for (std::size_t i = 0; i + 1 < way.refs.size(); ++i) {
                const long long a = get_node(way.refs[i]);
                const long long b = get_node(way.refs[i + 1]);
                if (a < 0 || b < 0) continue;
                RoadEdge edge{};
                edge.id = edge_id++;
                edge.from = static_cast<std::size_t>(a);
                edge.to = static_cast<std::size_t>(b);
                edge.road_class = way.highway;
                edge.speed_limit = speed;
                edge.polyline = {graph.nodes[a].pos, graph.nodes[b].pos};
                edge.length_meters = distance(graph.nodes[a].pos, graph.nodes[b].pos);
                graph.edges.push_back(edge);
            }
        }

        graph.build_adjacency();
        return graph;
    }

    void osm_importer::save_binary(const RoadGraph &graph, const std::filesystem::path &out) {
        std::ofstream os(out, std::ios::binary);
        if (!os) throw std::runtime_error("Cannot write graph binary: " + out.string());

        const auto w_u64 = [&](std::uint64_t v) { os.write(reinterpret_cast<char *>(&v), 8); };
        const auto w_dbl = [&](double v) { os.write(reinterpret_cast<char *>(&v), 8); };

        const char magic[4] = {'R', 'G', 'B', '1'};
        os.write(magic, 4);
        w_u64(graph.nodes.size());
        for (const auto &node : graph.nodes) {
            w_u64(node.id);
            w_dbl(node.pos.e);
            w_dbl(node.pos.n);
            w_dbl(node.lat);
            w_dbl(node.lon);
        }
        w_u64(graph.edges.size());
        for (const auto &e : graph.edges) {
            w_u64(e.id);
            w_u64(e.from);
            w_u64(e.to);
            w_dbl(e.length_meters);
            w_dbl(e.speed_limit);
            w_u64(e.road_class.size());
            os.write(e.road_class.data(), static_cast<std::streamsize>(e.road_class.size()));
            w_u64(e.polyline.size());
            for (const auto &p : e.polyline) { w_dbl(p.e); w_dbl(p.n); }
        }
    }

    RoadGraph osm_importer::load_binary(const std::filesystem::path &in) {
        std::ifstream is(in, std::ios::binary);
        if (!is) throw std::runtime_error("Cannot read graph binary: " + in.string());

        const auto r_u64 = [&]() { std::uint64_t v = 0; is.read(reinterpret_cast<char *>(&v), 8); return v; };
        const auto r_dbl = [&]() { double v = 0; is.read(reinterpret_cast<char *>(&v), 8); return v; };

        char magic[4];
        is.read(magic, 4);
        if (magic[0] != 'R' || magic[1] != 'G' || magic[2] != 'B' || magic[3] != '1') {
            throw std::runtime_error("Bad road-graph binary magic: " + in.string());
        }
        RoadGraph graph;
        const std::uint64_t nn = r_u64();
        graph.nodes.reserve(nn);
        for (std::uint64_t i = 0; i < nn; ++i) {
            RoadNode node{};
            node.id = r_u64();
            node.pos.e = r_dbl();
            node.pos.n = r_dbl();
            node.lat = r_dbl();
            node.lon = r_dbl();
            graph.nodes.push_back(node);
        }
        const std::uint64_t ne = r_u64();
        graph.edges.reserve(ne);
        for (std::uint64_t i = 0; i < ne; ++i) {
            RoadEdge e{};
            e.id = r_u64();
            e.from = r_u64();
            e.to = r_u64();
            e.length_meters = r_dbl();
            e.speed_limit = r_dbl();
            const std::uint64_t cls_len = r_u64();
            e.road_class.resize(cls_len);
            is.read(e.road_class.data(), static_cast<std::streamsize>(cls_len));
            const std::uint64_t pn = r_u64();
            e.polyline.resize(pn);
            for (std::uint64_t k = 0; k < pn; ++k) { e.polyline[k].e = r_dbl(); e.polyline[k].n = r_dbl(); }
            graph.edges.push_back(e);
        }
        graph.build_adjacency();
        return graph;
    }

}

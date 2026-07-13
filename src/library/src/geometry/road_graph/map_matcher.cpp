// Copyright (C) 2025 Kartik Vadhawana
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

#include "geometry/road_graph/map_matcher.hpp"

#include <algorithm>
#include <cmath>

namespace map_matching_2::geometry::road_graph {

    Point2D map_matcher::project_point_to_segment(
            const Point2D &p, const Point2D &a, const Point2D &b, double &out_dist) {
        const double dx = b.e - a.e;
        const double dy = b.n - a.n;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 1e-12) {
            t = ((p.e - a.e) * dx + (p.n - a.n) * dy) / len2;
            t = std::clamp(t, 0.0, 1.0);
        }
        Point2D proj{a.e + t * dx, a.n + t * dy};
        out_dist = distance(p, proj);
        return proj;
    }

    MatchResult map_matcher::match(const RoadGraph &graph,
            const std::vector<io::track::KittiOxtsFrame> &frames) const {
        MatchResult result{};
        result.total_points = frames.size();
        result.points.resize(frames.size());
        if (frames.empty() || graph.edges.empty()) {
            return result;
        }

        // Per GPS point: candidate edges within the search radius, each with
        // its snapped point, perpendicular distance and emission log-prob.
        struct Candidate {
            std::size_t edge_index;
            Point2D snapped;
            double dist;
            double emission_log; // log N(dist; 0, sigma_z)
        };
        std::vector<std::vector<Candidate>> cands(frames.size());

        const double inv_2sig2 = 1.0 / (2.0 * _params.sigma_z_m * _params.sigma_z_m);
        for (std::size_t i = 0; i < frames.size(); ++i) {
            const Point2D gp{frames[i].easting, frames[i].northing};
            std::vector<Candidate> local;
            for (std::size_t e = 0; e < graph.edges.size(); ++e) {
                const auto &edge = graph.edges[e];
                if (edge.polyline.size() < 2) continue;
                double best_d = std::numeric_limits<double>::max();
                Point2D best_proj{};
                for (std::size_t k = 0; k + 1 < edge.polyline.size(); ++k) {
                    double d;
                    Point2D proj = project_point_to_segment(
                            gp, edge.polyline[k], edge.polyline[k + 1], d);
                    if (d < best_d) { best_d = d; best_proj = proj; }
                }
                if (best_d <= _params.search_radius_m) {
                    Candidate c{};
                    c.edge_index = e;
                    c.snapped = best_proj;
                    c.dist = best_d;
                    c.emission_log = -best_d * best_d * inv_2sig2;
                    local.push_back(c);
                }
            }
            std::sort(local.begin(), local.end(),
                    [](const Candidate &x, const Candidate &y) { return x.dist < y.dist; });
            if (local.size() > _params.max_candidates) local.resize(_params.max_candidates);
            cands[i] = std::move(local);
        }

        // Viterbi over candidate states. Transition log-prob penalizes the
        // difference between the GPS step length and the on-road distance
        // between successive snapped points (great-circle route approx).
        const double neg_inf = -std::numeric_limits<double>::infinity();
        std::vector<std::vector<double>> vit(frames.size());
        std::vector<std::vector<int>> back(frames.size());

        // find first point that actually has candidates to seed the trellis
        for (std::size_t i = 0; i < frames.size(); ++i) {
            vit[i].assign(cands[i].size(), neg_inf);
            back[i].assign(cands[i].size(), -1);
        }
        bool seeded = false;
        for (std::size_t j = 0; j < cands[0].size(); ++j) {
            vit[0][j] = cands[0][j].emission_log;
            seeded = true;
        }
        std::size_t prev_layer = 0;
        (void) seeded;

        for (std::size_t i = 1; i < frames.size(); ++i) {
            if (cands[i].empty()) continue;
            if (cands[prev_layer].empty()) { prev_layer = i; continue; }

            const double gps_step = distance(
                    {frames[prev_layer].easting, frames[prev_layer].northing},
                    {frames[i].easting, frames[i].northing});

            for (std::size_t j = 0; j < cands[i].size(); ++j) {
                double best = neg_inf;
                int best_k = -1;
                for (std::size_t k = 0; k < cands[prev_layer].size(); ++k) {
                    if (vit[prev_layer][k] == neg_inf) continue;
                    const double route = distance(cands[prev_layer][k].snapped, cands[i][j].snapped);
                    const double dt = std::fabs(gps_step - route);
                    const double trans_log = -dt / _params.beta_m;
                    const double score = vit[prev_layer][k] + trans_log + cands[i][j].emission_log;
                    if (score > best) { best = score; best_k = static_cast<int>(k); }
                }
                if (best_k >= 0) {
                    vit[i][j] = best;
                    back[i][j] = best_k;
                } else {
                    // no viable predecessor; restart local chain
                    vit[i][j] = cands[i][j].emission_log;
                    back[i][j] = -1;
                }
            }
            prev_layer = i;
        }

        // Backtrace from the last layer that has candidates.
        long long last = -1;
        for (long long i = static_cast<long long>(frames.size()) - 1; i >= 0; --i) {
            if (!cands[i].empty()) { last = i; break; }
        }
        if (last >= 0) {
            std::size_t best_j = 0;
            double best_v = neg_inf;
            for (std::size_t j = 0; j < vit[last].size(); ++j) {
                if (vit[last][j] > best_v) { best_v = vit[last][j]; best_j = j; }
            }
            long long i = last;
            long long j = static_cast<long long>(best_j);
            while (i >= 0 && j >= 0) {
                const auto &c = cands[i][j];
                MatchedPoint mp{};
                mp.matched = true;
                mp.edge_index = c.edge_index;
                mp.snapped = c.snapped;
                mp.gps_distance_m = c.dist;
                mp.confidence = std::exp(c.emission_log); // in (0,1]
                result.points[i] = mp;
                const int pk = back[i][j];
                --i;
                if (i < 0) break;
                if (pk < 0) {
                    // chain restarted here; pick best candidate of layer i
                    if (cands[i].empty()) { j = -1; }
                    else {
                        std::size_t bj = 0; double bv = neg_inf;
                        for (std::size_t x = 0; x < vit[i].size(); ++x)
                            if (vit[i][x] > bv) { bv = vit[i][x]; bj = x; }
                        j = static_cast<long long>(bj);
                    }
                } else {
                    j = pk;
                }
            }
        }

        // Fill any unmatched points (no candidates) and tally statistics.
        double conf_sum = 0.0;
        for (std::size_t i = 0; i < frames.size(); ++i) {
            if (result.points[i].matched) {
                ++result.matched_points;
                conf_sum += result.points[i].confidence;
            } else {
                ++result.unmatched_points;
            }
        }
        result.matching_rate = result.total_points == 0
                ? 0.0 : static_cast<double>(result.matched_points) / result.total_points;
        result.mean_confidence = result.matched_points == 0
                ? 0.0 : conf_sum / result.matched_points;
        return result;
    }

}

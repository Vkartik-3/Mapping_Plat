#!/usr/bin/env python3
# Copyright (C) 2025 Kartik Vadhawana
#
# Render the OSM road network, the raw GPS trace, the map-matched trajectory
# and intersections onto an interactive folium map (self-contained map.html).
#
# Inputs (all CSV, lon/lat in WGS84):
#   --edges edges.csv       columns: lon1,lat1,lon2,lat2   (one road segment/row)
#   --gps   gps.csv         columns: lon,lat               (raw GPS points)
#   --matched matched.csv   columns: lon,lat               (snapped trajectory)
#   --nodes nodes.csv       columns: lon,lat               (intersections, optional)
# Or run with --demo to synthesize a small Karlsruhe example and prove output.

import argparse
import csv
import math
import os

import folium


def read_pairs(path):
    rows = []
    with open(path, newline="") as f:
        r = csv.reader(f)
        for row in r:
            if not row or row[0].startswith("#"):
                continue
            try:
                rows.append([float(x) for x in row])
            except ValueError:
                continue  # skip header rows (e.g. "lon,lat")
    return rows


def demo_data():
    """L-shaped drive near Karlsruhe: east then north, matching the C++ demo."""
    lat0, lon0 = 49.0, 8.4
    m_per_lat = 6371000.0 * math.pi / 180.0
    m_per_lon = m_per_lat * math.cos(math.radians(lat0))
    gps, matched, edges = [], [], []
    for i in range(40):
        lon = lon0 + (i * 5.0) / m_per_lon
        gps.append([lon, lat0 + (0.6 - (i % 3)) * 0.5 / m_per_lat])  # jittered
        matched.append([lon, lat0])
    east_fixed = 39 * 5.0
    for i in range(1, 41):
        lat = lat0 + (i * 5.0) / m_per_lat
        lon = lon0 + east_fixed / m_per_lon
        gps.append([lon + (0.5 - (i % 3)) * 0.5 / m_per_lon, lat])
        matched.append([lon, lat])
    corner_lon = lon0 + east_fixed / m_per_lon
    top_lat = lat0 + (40 * 5.0) / m_per_lat
    edges = [
        [lon0, lat0, corner_lon, lat0],
        [corner_lon, lat0, corner_lon, top_lat],
    ]
    nodes = [[corner_lon, lat0]]  # the intersection/corner
    return edges, gps, matched, nodes


def build_map(edges, gps, matched, nodes, out_path, lowconf=None, disconnected=None):
    lowconf = lowconf or []
    disconnected = disconnected or []
    all_lat = [p[1] for p in gps] + [p[1] for p in matched]
    all_lon = [p[0] for p in gps] + [p[0] for p in matched]
    center = [sum(all_lat) / len(all_lat), sum(all_lon) / len(all_lon)]
    m = folium.Map(location=center, zoom_start=16, tiles="OpenStreetMap")

    # OSM road graph (blue).
    for e in edges:
        folium.PolyLine([[e[1], e[0]], [e[3], e[2]]], color="blue",
                        weight=4, opacity=0.6, tooltip="OSM road edge").add_to(m)

    # Raw KITTI GPS trajectory (red points).
    for p in gps:
        folium.CircleMarker([p[1], p[0]], radius=2, color="red",
                            fill=True, fill_opacity=0.8, tooltip="raw GPS").add_to(m)

    # HMM-matched trajectory (green line) + snapped points (green dots).
    if matched:
        folium.PolyLine([[p[1], p[0]] for p in matched], color="green",
                        weight=3, opacity=0.9, tooltip="HMM matched trajectory").add_to(m)
        for p in matched:
            folium.CircleMarker([p[1], p[0]], radius=2, color="green",
                                fill=True, fill_opacity=0.9, tooltip="snapped point").add_to(m)

    # Intersections (orange stars).
    for n in nodes:
        folium.Marker([n[1], n[0]], icon=folium.Icon(color="orange", icon="star"),
                      tooltip="intersection").add_to(m)

    # Low-confidence matches (purple).
    for p in lowconf:
        folium.CircleMarker([p[1], p[0]], radius=5, color="purple",
                            fill=True, fill_opacity=0.7, tooltip="low-confidence match").add_to(m)

    # Disconnected transitions (black x markers).
    for p in disconnected:
        folium.Marker([p[1], p[0]], icon=folium.Icon(color="black", icon="remove"),
                      tooltip="disconnected transition").add_to(m)

    m.save(out_path)
    return out_path


def main():
    ap = argparse.ArgumentParser(description="Visualize road network + matched GPS.")
    ap.add_argument("--edges")
    ap.add_argument("--gps")
    ap.add_argument("--matched")
    ap.add_argument("--nodes")
    ap.add_argument("--lowconf")
    ap.add_argument("--disconnected")
    ap.add_argument("--workflow-dir",
                    help="directory of CSVs produced by kitti_osm_workflow "
                         "(gps.csv, edges.csv, matched.csv, lowconf.csv, disconnected.csv)")
    ap.add_argument("--demo", action="store_true", help="synthesize demo data")
    ap.add_argument("--out", default="map.html")
    args = ap.parse_args()

    lowconf, disconnected = [], []
    if args.workflow_dir:
        d = args.workflow_dir
        edges = read_pairs(os.path.join(d, "edges.csv"))
        gps = read_pairs(os.path.join(d, "gps.csv"))
        matched = read_pairs(os.path.join(d, "matched.csv"))
        lc = os.path.join(d, "lowconf.csv")
        dc = os.path.join(d, "disconnected.csv")
        lowconf = read_pairs(lc) if os.path.exists(lc) else []
        disconnected = read_pairs(dc) if os.path.exists(dc) else []
        nodes = []
    elif args.demo or not (args.edges and args.gps):
        edges, gps, matched, nodes = demo_data()
    else:
        edges = read_pairs(args.edges)
        gps = read_pairs(args.gps)
        matched = read_pairs(args.matched) if args.matched else []
        nodes = read_pairs(args.nodes) if args.nodes else []
        lowconf = read_pairs(args.lowconf) if args.lowconf else []
        disconnected = read_pairs(args.disconnected) if args.disconnected else []

    out = build_map(edges, gps, matched, nodes, args.out, lowconf, disconnected)
    print(f"wrote {out} ({os.path.getsize(out)} bytes): "
          f"{len(edges)} edges, {len(gps)} gps pts, {len(matched)} matched, "
          f"{len(nodes)} intersections, {len(lowconf)} low-conf, "
          f"{len(disconnected)} disconnected")


if __name__ == "__main__":
    main()

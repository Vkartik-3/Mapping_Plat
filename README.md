# HD Map Pipeline

**A high-performance C++20 pipeline for HD-map and sensor-data engineering: GPS/IMU
trajectory analysis, road-network graph construction, LiDAR point-cloud
processing, sensor-data integrity validation, and reproducible benchmarking —
validated end-to-end on the real KITTI raw dataset.**

The pipeline ingests raw autonomous-driving sensor logs (KITTI OXTS GPS/IMU +
Velodyne LiDAR), turns GPS traces into road geometry, matches trajectories onto a
road-network graph, indexes and validates 3D point clouds, and measures every
stage with a real-numbers benchmark harness and a GoogleTest suite.

---

## Table of contents

- [What it does](#what-it-does)
- [Architecture](#architecture)
- [Components](#components)
- [Building](#building)
- [Running](#running)
- [Results (real KITTI)](#results-real-kitti)
- [Engineering notes & notable issues](#engineering-notes--notable-issues)
- [Repository layout](#repository-layout)
- [License & attribution](#license--attribution)

---

## What it does

The work maps directly onto the core problems of HD-mapping / sensor-fusion teams:

| Capability | Implementation |
|---|---|
| GPS/IMU trajectory analysis | KITTI OXTS reader → local ENU frame (Haversine), IMU + accuracy fields |
| Road centerline extraction | Trace-clustering (Schroedl / Biagioni) on GPS heading + proximity |
| Road-network graph construction | OSM `.osm` → attributed road graph (length, speed, class, polyline) |
| Map matching (trace → map) | HMM / Viterbi matcher (Newson–Krumm) snapping GPS onto the graph |
| Efficient spatial retrieval | nanoflann 3D k-d tree (NN / kNN / radius) over point clouds |
| LiDAR processing | RANSAC ground-plane extraction + ground/obstacle classification |
| Sensor-data integrity | CRC32 per-frame checks + an 8-check LiDAR frame validator |
| Storage & I/O | Binary road-graph serialization; KITTI Velodyne `.bin` decode |
| Benchmarking | Google Benchmark harness (15 benchmarks, incl. real-KITTI runs) |
| Visualization | Python (folium road network, matplotlib point cloud) |

Everything is native **C++20**, `std`-only in the hot paths (no heavy runtime
dependencies), and builds without the upstream matching engine's Boost/Osmium
stack via self-contained `test/` and `bench/` CMake projects.

---

## Architecture

```
                         KITTI raw sequence (2011_09_26_drive_XXXX_sync)
                          ├── oxts/data/*.txt        (GPS/IMU, 10 Hz)
                          └── velodyne_points/data/*.bin  (LiDAR frames)
                                        │
              ┌─────────────────────────┴──────────────────────────┐
              ▼                                                      ▼
   ┌───────────────────────┐                          ┌──────────────────────────┐
   │  OXTS GPS/IMU reader   │                          │  Velodyne .bin reader     │
   │  lat/lon/alt → ENU     │                          │  x,y,z,intensity decode   │
   │  (Haversine anchor)    │                          │  + CRC32 integrity        │
   └───────────┬───────────┘                          └────────────┬─────────────┘
               │                                                     │
      ┌────────┴─────────┐                             ┌─────────────┴─────────────┐
      ▼                  ▼                             ▼                           ▼
┌───────────┐   ┌──────────────────┐         ┌────────────────┐        ┌────────────────────┐
│ Centerline │   │  HMM map matcher │         │ nanoflann k-d  │        │  RANSAC ground +   │
│ extraction │   │  (GPS → OSM)     │◄──OSM──►│ tree (3D NN)   │        │  frame validator   │
└───────────┘   └──────────────────┘         └────────────────┘        └────────────────────┘
                          │                                                     │
              ┌───────────┴────────────┐                                        │
              ▼                        ▼                                        ▼
      Road-network graph        Matched trajectory                    Ground/obstacle +
   (nodes, edges, Dijkstra)     (snapped, confidence)                 integrity report
                          │
                          ▼
        Benchmarks (Google Benchmark) · Tests (GoogleTest) · Python visualization
```

`KittiSequence` synchronizes OXTS and Velodyne frames by index and reports
per-sequence statistics (frame count, GPS-fix count, CRC failures, mean
points/frame, GPS bounding box).

---

## Components

### Data ingestion

- **KITTI OXTS reader** — [`kitti_oxts_importer`](src/library/src/types/io/track/kitti_oxts_importer.cpp).
  Parses one `.txt` per frame (30 fields), validates ranges, skips malformed
  frames, and converts geodetic coordinates to a local ENU frame anchored at the
  first frame using the Haversine formula. Exposes position, velocity (n/e/f),
  IMU (accel + angular rate), and GPS accuracy/quality fields.
- **KITTI Velodyne reader** — [`kitti_lidar_reader`](src/library/src/types/io/lidar/kitti_lidar_reader.cpp).
  Decodes raw little-endian `float32 (x, y, z, intensity)`, enforces the 16-byte
  stride, screens coordinate/intensity ranges, computes a **CRC32** over the raw
  bytes, and verifies it against an optional `.bin.crc` sidecar.
- **CRC32** — [`util/crc32.hpp`](src/library/include/util/crc32.hpp).
  Header-only, `constexpr`, reflected polynomial `0xEDB88320` (matches
  zlib/PNG/Ethernet). Verified against the standard check value
  (`CRC32("123456789") = 0xCBF43926`).
- **Synchronized store** — [`KittiSequence`](src/library/include/types/io/kitti_sequence.hpp).
  Pairs OXTS ↔ LiDAR by frame index and aggregates sequence statistics.

### Road network from GPS + OSM

- **Centerline extraction** — [`centerline_extractor`](src/library/src/geometry/centerline/centerline_extractor.cpp).
  Bins trajectory points by heading (10° buckets), clusters spatially within each
  bucket, connects centroids into ordered segments, and marks intersections where
  differently-headed clusters meet — the trace-mining approach of Schroedl et al.
  / Biagioni & Eriksson.
- **OSM road importer** — [`osm_importer`](src/library/src/geometry/road_graph/osm_importer.cpp).
  Dependency-free `.osm` XML parser → attributed [`RoadGraph`](src/library/include/geometry/road_graph/road_graph.hpp)
  (length, speed limit, highway class, polyline) in the same ENU frame as the GPS,
  with binary save/load.
- **HMM map matcher** — [`map_matcher`](src/library/src/geometry/road_graph/map_matcher.cpp).
  Newson–Krumm hidden-Markov matching: Gaussian emission from perpendicular
  GPS-to-edge distance, transition penalty on GPS-step vs. on-road distance, and a
  Viterbi decode returning matched edges, snapped points, and per-point confidence.
- **Graph algorithms** — [`graph_algorithms.hpp`](src/library/include/geometry/road_graph/graph_algorithms.hpp):
  Dijkstra shortest path (edge weight = length) and nearest-edge spatial query.

### Spatial indexing & LiDAR

- **nanoflann k-d tree** — [`geometry/index/kdtree.hpp`](src/library/include/geometry/index/kdtree.hpp).
  3D index over LiDAR points: nearest-neighbour, kNN, and radius search, with
  incremental rebuild. Wraps the `nanoflann` submodule.
- **RANSAC ground extraction** — [`ground_extractor`](src/library/src/geometry/lidar/ground_extractor.cpp).
  Random 3-point plane sampling with inlier counting; classifies ground vs.
  obstacle and reports ground ratio, plane normal, and inlier/outlier counts. The
  result type is a standard-layout POD for ABI stability.
- **Frame validator** — [`frame_validator.hpp`](src/library/include/geometry/lidar/frame_validator.hpp).
  Eight integrity checks per frame: CRC, point-count bounds, NaN/Inf screening,
  range validity, azimuth coverage (density), intensity distribution, and
  ground-extraction success → a structured pass/fail report.

### Quality & tooling

- **GoogleTest suite** — [`test/`](test/): **39 tests** across CRC32, KITTI
  readers, k-d tree, ground extraction, frame validation, centerline, and map
  matching. Builds without Boost/Osmium.
- **Google Benchmark harness** — [`bench/`](bench/): **15 benchmarks** including
  four that run against the **real KITTI sequence** (`*_real`) and skip cleanly
  when the dataset is absent.
- **CI** — [`.github/workflows/ci.yml`](.github/workflows/ci.yml): build, test
  (ctest), benchmarks (JSON artifact), and an AddressSanitizer + UBSan job.
- **Visualization** — [`python/`](python/): `visualize_map.py` (folium: road
  edges, raw GPS, matched trajectory, intersections) and
  `visualize_pointcloud.py` (matplotlib: ground/obstacle + intensity).

---

## Building

Requirements: a C++20 compiler (GCC 13+, Clang 15+), CMake ≥ 3.16, and the
submodules (`nanoflann`, `googletest`, `benchmark`):

```bash
git submodule update --init --recursive
```

### Tests

```bash
cmake -S test -B build/test -DCMAKE_BUILD_TYPE=Release
cmake --build build/test -j
ctest --test-dir build/test --output-on-failure
```

### Benchmarks

```bash
cmake -S bench -B build/bench -DCMAKE_BUILD_TYPE=Release
cmake --build build/bench -j
./build/bench/hdmap_bench
```

Both are self-contained CMake projects: they compile the pipeline sources plus
the bundled `googletest`/`benchmark`/`nanoflann` submodules — no system Boost or
Osmium required.

---

## Running

### Against the real KITTI dataset

Point `KITTI_PATH` at a KITTI raw sequence directory (containing `oxts/data/` and
`velodyne_points/data/`); it defaults to the standard EC2 path if unset:

```bash
KITTI_PATH=/path/to/2011_09_26/2011_09_26/2011_09_26_drive_0001_sync \
  ./build/bench/hdmap_bench --benchmark_filter='.*_real'
```

Real-data benchmarks print `[SKIP] Real KITTI benchmarks: path not found` and
fall back gracefully when the dataset is absent (e.g. in CI).

### Visualization

```bash
pip install -r python/requirements.txt
python python/visualize_map.py --demo --out map.html
python python/visualize_pointcloud.py --bin frame.bin --out frame.png
```

---

## Results (real KITTI)

Measured on the real `2011_09_26_drive_0001_sync` sequence (108 frames) on EC2
(GCC 13.3, `-O3`). Full tables — including the local/synthetic reference numbers,
kept clearly separate — are in [docs/BENCHMARKS.md](docs/BENCHMARKS.md).

| Benchmark (real KITTI) | Throughput | Time/frame | Notes |
|---|---|---|---|
| `kitti_oxts_parse_real` | 97,576 frames/s | 0.010 ms | 108 GPS/IMU frames |
| `kitti_lidar_read_real` | 100.1 frames/s | 9.99 ms | 108 Velodyne frames |
| `ground_extraction_real` | 63.6 frames/s | 15.73 ms | `mean_ground_ratio = 0.6676` |
| `frame_validate_real` | 55.2 frames/s | 18.10 ms | `108 / 108` frames pass (100%) |

Correctness: CRC32 matches the standard check value; RANSAC recovers the ground
plane; k-d tree NN of an existing point returns distance 0; **39/39** GoogleTests
pass; AddressSanitizer + UBSan clean.

---

## Engineering notes & notable issues

Real problems solved during development — the kind of ABI/optimizer/measurement
issues that only surface on real hardware and real data:

- **Corrupted benchmark counter (GCC 13.3, `-O3`) — misuse of `DoNotOptimize`.**
  On EC2, `mean_ground_ratio` read back as `0` (and, in early forms, garbage
  denormals) while the extractor computed a correct value. A GDB **hardware
  watchpoint** on the field traced the zero-write to the *caller*: an untimed
  statistics loop passed a non-const lvalue to `benchmark::DoNotOptimize(...)` and
  then read the field back. The non-const `DoNotOptimize(Tp&)` overload uses an
  `"+m,r"` (read-write) asm constraint, so the compiler is entitled to clobber the
  operand. **Fix:** drop the barrier in untimed passes (values already feed real
  arithmetic, so there is no dead-code-elision risk); keep it only in timed loops
  where the result is discarded. This was *not* a layout, flag, or compiler bug —
  all were ruled out by matching offsets/flags and the watchpoint evidence.

- **ABI-robust result type.** `GroundResult` was reduced to an all-scalar,
  standard-layout, trivially-copyable POD (dropping an unused `std::vector<bool>`)
  so its return-by-value ABI is fixed across every translation unit, compiler, and
  standard library — removing a whole class of cross-TU layout hazards.

- **Cross-toolchain include hygiene.** libc++ (local) resolved several standard
  includes transitively that libstdc++ (CI/EC2) does not; explicit includes were
  added after CI caught the difference.

- **Benchmark counters vs. iteration counts.** Dataset-level counters
  (`mean_ground_ratio`, `pass_count`/`total_count`) are computed once over a
  single pass into initialized values, so they are stable under Google Benchmark's
  repetition/aggregation rather than being multiplied by iteration counts.

---

## Repository layout

```
src/library/           Core pipeline library (C++20)
  include/, src/
    types/io/          KITTI OXTS + Velodyne readers, sequence store
    geometry/          centerline, road_graph, index (k-d tree), lidar
    util/              crc32, synthetic data generators
  ... plus the road-matching engine (network graph, routing, HMM/MDP)
bench/                 Google Benchmark harness (15 benchmarks)
test/                  GoogleTest suite (39 tests)
python/                folium + matplotlib visualization
docs/                  BENCHMARKS.md (results), MATCHING_ENGINE.md (engine docs)
third_party/           nanoflann, googletest, benchmark (submodules)
.github/workflows/     CI (build / test / bench / asan+ubsan)
```

---

## License & attribution

This project is licensed under the **GNU Affero General Public License v3.0**
(see [LICENSE.md](LICENSE.md)).

The road-matching engine under `src/library` and `src/app` derives from
**map-matching-2** by Adrian Wöltche (https://github.com/iisys-hof/map-matching-2),
which is AGPL-3.0; its original documentation is preserved in
[docs/MATCHING_ENGINE.md](docs/MATCHING_ENGINE.md). All extensions described
above — KITTI ingestion, centerline extraction, the OSM road-graph and HMM
matcher, the nanoflann k-d tree, RANSAC ground extraction, the frame validator,
CRC32, the benchmark/test/CI/visualization tooling — were added by
**Kartik Vadhawana**. Bundled third-party libraries (`nanoflann`, `googletest`,
`benchmark`) are under their respective licenses.

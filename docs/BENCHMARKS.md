# HD Map Pipeline — Benchmark Results

Measured numbers for the pipeline extensions (KITTI ingestion, road-graph,
LiDAR, k-d tree). Two environments are reported and **kept clearly separate**:

- **EC2 (real KITTI data)** — the canonical results, run against the real
  `2011_09_26_drive_0001_sync` sequence (108 frames).
- **Local (synthetic data)** — a macOS/clang developer box, used for
  correctness checks and relative comparison only.

Never mix the two: they use different CPUs, compilers, and (for the real rows)
different input data.

---

## Environment

| | EC2 (canonical) | Local (reference) |
|---|---|---|
| OS | Ubuntu (EC2) | macOS (darwin) |
| Compiler | GCC 13.3.0 | Apple clang |
| Build type | Release (`-O3 -DNDEBUG`) | Release (`-O3`) |
| C++ standard | C++20 | C++20 |
| CPU | 4 × ~3.18 GHz | 8-core Apple Silicon |
| KITTI sequence | `2011_09_26_drive_0001_sync` (108 frames) | synthetic |
| Source revision | `0e7be08` | `0e7be08` |

Reproduce on EC2:

```bash
cmake -S bench -B build_ec2 -DCMAKE_BUILD_TYPE=Release
cmake --build build_ec2 -j
KITTI_PATH=/home/ubuntu/kitti/2011_09_26/2011_09_26/2011_09_26_drive_0001_sync \
  ./build_ec2/hdmap_bench --benchmark_min_time=0.10s
```

Real-data benchmarks skip cleanly with `[SKIP] Real KITTI benchmarks: path not
found` when `KITTI_PATH` is unset/absent (e.g. in CI).

---

## Canonical results — EC2, real KITTI (108 frames)

Values taken from the machine-readable JSON run (full precision).

| Benchmark | Throughput | Time/frame | Notes |
|---|---|---|---|
| `kitti_oxts_parse_real` | 97,576.65 frames/s | 0.01025 ms | 108 OXTS GPS/IMU frames |
| `kitti_lidar_read_real` | 100.119 frames/s | 9.989 ms | 108 Velodyne `.bin` frames |
| `ground_extraction_real` | 63.569 frames/s | 15.732 ms | RANSAC; `mean_ground_ratio = 0.6675539843` |
| `frame_validate_real` | 55.248 frames/s | 18.101 ms | 8-check validator; `pass_count = 108 / total = 108` (100%) |

Full-sequence wall times (real): OXTS parse ≈ 1.11 ms, LiDAR read ≈ 1078.75 ms,
ground extraction ≈ 1699.07 ms, frame validation ≈ 1954.89 ms.

### Full EC2 console run (all benchmarks; synthetic rows included for context)

```text
Run on (4 X 3178.99 MHz CPU s)
---------------------------------------------------------------------------------
Benchmark                       Time             CPU   Iterations UserCounters...
---------------------------------------------------------------------------------
kitti_oxts_parse/200      1370292 ns      1369814 ns          101 items_per_second=146.005k/s
kitti_oxts_parse_real     1104673 ns      1104680 ns          127 items_per_second=97.7658k/s
kitti_lidar_read_1k          93.2 ms         93.2 ms            2 items_per_second=10.7296k/s
kitti_lidar_read_real        1079 ms         1079 ms            1 items_per_second=100.132/s
crc32_1frame              4765150 ns      4765163 ns           29 bytes_per_second=384.259Mi/s
ground_extraction_1          15.9 ms         15.9 ms            9
frame_validate_100           2015 ms         2015 ms            1 items_per_second=49.6372/s
ground_extraction_real       1694 ms         1694 ms            1 frames=108 items_per_second=63.7463/s mean_ground_ratio=0.667554
frame_validate_real          1948 ms         1947 ms            1 frames=108 items_per_second=55.4571/s pass_count=108 total_count=108
graph_construct_osm           322 us          322 us          431
graph_dijkstra_1k            27.8 ms         27.8 ms            5 items_per_second=35.9848k/s
rtree_knn_1k                 16.1 ms         16.1 ms            9 items_per_second=62.2525k/s
map_match_sequence          0.027 ms        0.027 ms         5224 items_per_second=7.41324M/s
kdtree_build_100k            29.5 ms         29.5 ms            5 items_per_second=3.38825M/s
kdtree_nn_1k                0.674 ms        0.674 ms          208 items_per_second=1.48412M/s
```

> Console vs. JSON times differ by run-to-run CPU-frequency variation; the JSON
> run is the precise source for the canonical table above.

---

## Reference results — Local, synthetic data (macOS/clang)

Relative-performance and correctness reference only; **not** comparable to the
EC2 real-data rows. Run at `-O2`/`-O3` on synthetic inputs.

| Benchmark | Result |
|---|---|
| `kitti_oxts_parse/200` | 2.95 ms |
| `kitti_lidar_read_1k` | 117 ms (8.74k frames/s) |
| `crc32_1frame` | 395 MiB/s |
| `ground_extraction_1` | 6.12 ms (synthetic 125k pts) |
| `graph_construct_osm` | 229 µs |
| `graph_dijkstra_1k` | 11.3 ms (88.8k q/s) |
| `rtree_knn_1k` | 6.57 ms (152k q/s) |
| `map_match_sequence` | 0.017 ms |
| `kdtree_build_100k` | 19.1 ms |
| `kdtree_nn_1k` | 0.446 ms (2.24M q/s) |

Correctness checks (local): CRC32 `"123456789"` → `0xCBF43926`; RANSAC recovers
planted ground ratio; k-d tree NN of an existing point → distance 0; GoogleTest
**39/39** pass; ASan+UBSan clean.

---

## Notes on the `mean_ground_ratio` bug (fixed in `0e7be08`)

On EC2 (GCC 13.3, `-O3`) the real-data counters were originally corrupt
(`mean_ground_ratio=0`, garbage doubles). A GDB hardware watchpoint traced the
write to the **caller**: the untimed statistics loop passed a non-const lvalue
to `benchmark::DoNotOptimize(g.ground_ratio)` and then read the field back. The
non-const `DoNotOptimize(Tp&)` overload uses an `"+m,r"` (read-write) asm
constraint, so the compiler may write into the operand — GCC stored `0`, which
the following `ratio_sum += g.ground_ratio` then consumed.

**Fix:** drop `DoNotOptimize` in the untimed statistics passes (the values are
consumed by real arithmetic, so there is no dead-code-elision risk). Timed loops
keep it, since their result is discarded and the barrier legitimately prevents
the measured call from being optimized away.

**Benchmark-authoring rule:** never pass a value to `DoNotOptimize` and then
read that same value expecting it unchanged. Read first, or don't add the
barrier when the value already feeds an observable computation.

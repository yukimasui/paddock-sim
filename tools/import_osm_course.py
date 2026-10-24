#!/usr/bin/env python3
"""Turns an OpenStreetMap road into a course data file for the simulator.

Usage:
    import_osm_course.py OSM_JSON OUTPUT_RS NAME [--reverse] [--cache ELEVATION_JSON]

OSM_JSON is an Overpass API JSON response holding one way (the road) with
its nodes, for example from the query

    [out:json];way["name"="日本平パークウェイ"](34.93,138.38,35.03,138.52);(._;>;);out body;

The road is converted to local metres (x east, y north) around its first
point, smoothed with a centripetal Catmull-Rom spline and resampled every
SPACING metres. Ground elevation comes from the Geospatial Information
Authority of Japan (GSI) elevation API, sampled every ELEVATION_SPACING
metres, smoothed and interpolated onto the points. --reverse drives the way
against its OSM direction. Elevations are cached in a JSON file so re-runs
don't query the API again.

Data: road geometry (c) OpenStreetMap contributors, ODbL; elevation from
GSI (国土地理院) tiles and API.
"""

import json
import math
import sys
import time
import urllib.request

SPACING = 5.0  # m between output points
ELEVATION_SPACING = 25.0  # m between elevation samples
ELEVATION_SMOOTHING = 75.0  # m, moving-average window for elevation
MEDIAN_REACH = 2  # samples either side for the outlier-removing median
EARTH_RADIUS = 6_371_000.0
GSI_URL = (
    "https://cyberjapandata2.gsi.go.jp/general/dem/scripts/getelevation.php"
    "?lon={lon}&lat={lat}&outtype=JSON"
)


def load_way(path):
    data = json.load(open(path, encoding="utf-8"))
    nodes = {e["id"]: e for e in data["elements"] if e["type"] == "node"}
    ways = [e for e in data["elements"] if e["type"] == "way"]
    if len(ways) != 1:
        sys.exit(f"expected one way, found {len(ways)}")
    return [(nodes[n]["lat"], nodes[n]["lon"]) for n in ways[0]["nodes"]]


def to_local(points, origin):
    lat0 = math.radians(origin[0])
    return [
        (
            math.radians(lon - origin[1]) * EARTH_RADIUS * math.cos(lat0),
            math.radians(lat - origin[0]) * EARTH_RADIUS,
        )
        for lat, lon in points
    ]


def to_geographic(point, origin):
    lat0 = math.radians(origin[0])
    lat = origin[0] + math.degrees(point[1] / EARTH_RADIUS)
    lon = origin[1] + math.degrees(point[0] / (EARTH_RADIUS * math.cos(lat0)))
    return lat, lon


def catmull_rom(points, steps=20):
    """Dense points on an open centripetal Catmull-Rom spline through points."""
    padded = [points[0]] + points + [points[-1]]
    dense = []
    for i in range(1, len(padded) - 2):
        p = padded[i - 1 : i + 3]
        t = [0.0]
        for k in range(1, 4):
            d = math.dist(p[k - 1], p[k]) ** 0.5
            t.append(t[-1] + max(d, 1e-6))

        def lerp(a, b, ta, tb, at):
            w = (at - ta) / (tb - ta)
            return (a[0] + (b[0] - a[0]) * w, a[1] + (b[1] - a[1]) * w)

        for s in range(steps):
            at = t[1] + (t[2] - t[1]) * s / steps
            a1 = lerp(p[0], p[1], t[0], t[1], at)
            a2 = lerp(p[1], p[2], t[1], t[2], at)
            a3 = lerp(p[2], p[3], t[2], t[3], at)
            b1 = lerp(a1, a2, t[0], t[2], at)
            b2 = lerp(a2, a3, t[1], t[3], at)
            dense.append(lerp(b1, b2, t[1], t[2], at))
    dense.append(points[-1])
    return dense


def resample(dense, spacing):
    out = [dense[0]]
    travelled = 0.0
    for a, b in zip(dense, dense[1:]):
        travelled += math.dist(a, b)
        if travelled >= spacing:
            out.append(b)
            travelled = 0.0
    if math.dist(out[-1], dense[-1]) > spacing * 0.5:
        out.append(dense[-1])
    return out


def cumulative(points):
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + math.dist(a, b))
    return distances


def fetch_elevation(lat, lon):
    url = GSI_URL.format(lat=f"{lat:.7f}", lon=f"{lon:.7f}")
    request = urllib.request.Request(url, headers={"User-Agent": "paddock-sim course import"})
    with urllib.request.urlopen(request, timeout=20) as response:
        value = json.load(response)["elevation"]
    if value == "-----":
        raise ValueError(f"no elevation at {lat}, {lon}")
    return float(value)


def main():
    args = sys.argv[1:]
    reverse = "--reverse" in args
    cache_path = None
    if "--cache" in args:
        cache_path = args[args.index("--cache") + 1]
    positional = [a for i, a in enumerate(args) if not a.startswith("--") and args[i - 1] != "--cache"]
    osm_path, output_path, name = positional[:3]

    geographic = load_way(osm_path)
    if reverse:
        geographic.reverse()
    origin = geographic[0]
    points = resample(catmull_rom(to_local(geographic, origin)), SPACING)
    distances = cumulative(points)
    length = distances[-1]

    cache = {}
    if cache_path:
        try:
            cache = json.load(open(cache_path))
        except FileNotFoundError:
            pass
    samples = []
    count = int(length // ELEVATION_SPACING) + 1
    for k in range(count + 1):
        at = min(k * ELEVATION_SPACING, length)
        i = max(j for j, d in enumerate(distances) if d <= at)
        lat, lon = to_geographic(points[i], origin)
        key = f"{lat:.6f},{lon:.6f}"
        if key not in cache:
            cache[key] = fetch_elevation(lat, lon)
            time.sleep(0.2)  # be gentle with the public API
        samples.append((distances[i], cache[key]))
    if cache_path:
        json.dump(cache, open(cache_path, "w"))

    # A median first removes single-sample outliers (a building or
    # embankment caught by the survey), then a moving average smooths out
    # bridges, cuttings and survey noise.
    medians = []
    for k, (d, _) in enumerate(samples):
        window = sorted(z for _, z in samples[max(0, k - MEDIAN_REACH) : k + MEDIAN_REACH + 1])
        medians.append((d, window[len(window) // 2]))
    half = ELEVATION_SMOOTHING / 2
    smoothed = []
    for d, _ in medians:
        window = [z for e, z in medians if abs(e - d) <= half]
        smoothed.append((d, sum(window) / len(window)))

    def elevation_at(d):
        for (d0, z0), (d1, z1) in zip(smoothed, smoothed[1:]):
            if d0 <= d <= d1:
                return z0 if d1 == d0 else z0 + (z1 - z0) * (d - d0) / (d1 - d0)
        return smoothed[-1][1]

    base = min(z for _, z in smoothed)
    lines = [
        f"//! {name}: course data generated by tools/import_osm_course.py.",
        "//!",
        "//! Road geometry (c) OpenStreetMap contributors, available under the Open",
        "//! Database License (ODbL). Elevation: Geospatial Information Authority of",
        "//! Japan (GSI) elevation data.",
        "//!",
        f"//! {len(points)} points, {length:.0f} m, x east and y north in metres from",
        f"//! the start, elevation in metres above the lowest point ({base:.1f} m above",
        "//! sea level).",
        "",
        "#[rustfmt::skip]",
        "pub const POINTS: &[[f32; 3]] = &[",
    ]
    for (x, y), d in zip(points, distances):
        lines.append(f"    [{x:.2f}, {y:.2f}, {elevation_at(d) - base:.2f}],")
    lines.append("];")
    open(output_path, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print(f"{len(points)} points, {length:.0f} m, climb {max(z for _, z in smoothed) - base:.0f} m")


if __name__ == "__main__":
    main()

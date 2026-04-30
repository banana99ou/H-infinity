"""Generate a parametric reference path in the rooftop track local frame,
project it to WGS84 lat/lon, fetch satellite tiles around the track, and
render the path overlaid on the imagery.

Track frame (set by memory/project_rooftop_track.md):
- Anchor: right-bottom corner of the rooftop rectangle.
- Local +x: along the long axis toward the "top" end (bearing ~42 deg E of N).
- Local +y: 90 deg CCW from +x (i.e., to the left of forward).

Output: PNG written to ./out/<name>.png plus a sidecar GeoJSON with the path.
"""

from __future__ import annotations

import argparse
import io
import json
import math
import os
from dataclasses import dataclass

import numpy as np
import requests
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon


# -----------------------------------------------------------------------------
# Track config (rooftop)
# -----------------------------------------------------------------------------

LAT0 = 37.61174497415274          # right-bottom corner anchor
LON0 = 126.99429176572984
BEARING_DEG = 42.0                 # long-axis bearing, east of true north
TRACK_LONG = 42.0                  # m
TRACK_SHORT = 10.0                 # m
EARTH_R = 6378137.0                # WGS84 semi-major (sphere approx is fine here)


# -----------------------------------------------------------------------------
# Path families. Each returns an (N,2) ndarray of waypoints in the local frame.
# -----------------------------------------------------------------------------

def path_oval(R=3.0, margin=1.0, n_arc=20, overshoot=1.0):
    """Open oval lap centered in the rectangle with a small overshoot at the end.

    Layout (local frame, +x along long axis):
        start --(straight)-- arc -- (straight back) -- arc -- (overshoot) end
    """
    long_in = TRACK_LONG - 2 * margin
    short_in = TRACK_SHORT - 2 * margin
    R = min(R, short_in / 2.0)
    L_str = long_in - 2 * R
    if L_str <= 0:
        raise ValueError(f"Oval R={R} too large for usable long={long_in:.2f}")

    cx_lo = margin + R
    cx_hi = margin + R + L_str
    cy_low = margin + R                    # lower lane center y
    cy_high = margin + R + (short_in - 2 * R)
    # If short_in == 2R, both lanes coincide -> shrink R a touch
    if cy_high - cy_low < 0.5:
        R *= 0.85
        cy_low = margin + R
        cy_high = TRACK_SHORT - margin - R
        cx_lo = margin + R
        cx_hi = TRACK_LONG - margin - R
        L_str = cx_hi - cx_lo

    pts = []
    # straight 1: low lane, +x
    pts.append((cx_lo, cy_low))
    pts.append((cx_hi, cy_low))
    # half arc at right end, going from (cx_hi, cy_low) up to (cx_hi, cy_high)
    th = np.linspace(-np.pi / 2, np.pi / 2, n_arc)
    arc_r = (cy_high - cy_low) / 2.0
    cy_mid = (cy_high + cy_low) / 2.0
    for t in th[1:]:
        pts.append((cx_hi + arc_r * np.cos(t), cy_mid + arc_r * np.sin(t)))
    # straight 2: high lane, -x
    pts.append((cx_lo, cy_high))
    # half arc at left end, going back down to start lane
    th2 = np.linspace(np.pi / 2, 3 * np.pi / 2, n_arc)
    for t in th2[1:]:
        pts.append((cx_lo + arc_r * np.cos(t), cy_mid + arc_r * np.sin(t)))
    # tiny overshoot in +x so the controller's 0.3 m end-stop doesn't trigger early
    pts.append((cx_lo + overshoot, cy_low))

    return np.array(pts, dtype=float)


def path_line(L=20.0, y=None):
    if y is None:
        y = TRACK_SHORT / 2.0
    return np.array([[1.0, y], [1.0 + L, y]], dtype=float)


def path_slalom(R=1.5, n_arcs=4, L1=4.0, L_mid=2.0, L_end=4.0, y=None,
                n_per_arc=12):
    """Alternating left/right arcs with straights between."""
    if y is None:
        y = TRACK_SHORT / 2.0
    pts = [(1.0, y)]
    x = 1.0 + L1
    pts.append((x, y))
    sign = +1
    for k in range(n_arcs):
        # half arc of radius R, sign alternating
        cx = x
        cy = y + sign * R
        # arc from (x, y) sweeping to (x + 2R, y) (a half-circle in the +x direction)
        # parameter t in [-pi/2, +pi/2] with center at (cx, cy) flipped
        t = np.linspace(-np.pi / 2 if sign > 0 else np.pi / 2,
                        np.pi / 2 if sign > 0 else 3 * np.pi / 2,
                        n_per_arc)
        # We want start at (x, y): cx + R cos(t0), cy + R sin(t0) = (x, y)
        # For sign=+1, center above: cos(t0)=0, sin(t0)=-1 -> t0 = -pi/2 (ok)
        # For sign=-1, center below: cos(t0)=0, sin(t0)=+1 -> t0 = +pi/2 (ok)
        for tt in t[1:]:
            pts.append((cx + R * np.cos(tt), cy + R * np.sin(tt)))
        x = pts[-1][0]
        sign *= -1
        if k < n_arcs - 1:
            x += L_mid
            pts.append((x, y))
    pts.append((x + L_end, y))
    return np.array(pts, dtype=float)


def path_sinusoid(L=30.0, A=1.5, n_waves=2, n=120, y0=None):
    if y0 is None:
        y0 = TRACK_SHORT / 2.0
    x = np.linspace(1.0, 1.0 + L, n)
    y = y0 + A * np.sin(2 * np.pi * n_waves * (x - x[0]) / L)
    return np.column_stack([x, y])


def path_step(L1=8.0, R=2.0, theta_deg=90.0, L2=10.0, y=None, n_arc=24,
              direction=+1):
    if y is None:
        y = 1.5
    theta = np.deg2rad(theta_deg)
    # straight 1
    pts = [(1.0, y), (1.0 + L1, y)]
    # arc
    cx = 1.0 + L1
    cy = y + direction * R
    t = np.linspace(-np.pi / 2 if direction > 0 else np.pi / 2,
                    -np.pi / 2 + direction * theta,
                    n_arc)
    for tt in t[1:]:
        pts.append((cx + R * np.cos(tt), cy + R * np.sin(tt)))
    # straight 2 along arc-exit tangent
    hx, hy = pts[-1]
    psi_end = direction * theta
    for k in (1,):
        pts.append((hx + L2 * np.cos(psi_end), hy + L2 * np.sin(psi_end)))
    return np.array(pts, dtype=float)


PATH_FAMILIES = {
    'oval': path_oval,
    'line': path_line,
    'slalom': path_slalom,
    'sinusoid': path_sinusoid,
    'step': path_step,
}


# -----------------------------------------------------------------------------
# Local <-> WGS84 projection
# -----------------------------------------------------------------------------

@dataclass
class Anchor:
    lat0: float
    lon0: float
    bearing_deg: float   # bearing of local +x axis, east of true north


def local_to_latlon(xy, anchor: Anchor):
    """Project local (x, y) m -> (lat, lon) deg via small-angle ENU.

    Local +x has compass bearing `anchor.bearing_deg` (east of north).
    Local +y is 90 deg CCW from +x (north-of-east relative to forward).
    """
    bearing = math.radians(anchor.bearing_deg)
    # ENU: east, north
    # Rotate local (x, y) into (east, north). Local +x axis in ENU =
    # (sin(bearing), cos(bearing)). Local +y axis = perpendicular CCW =
    # (-cos(bearing), sin(bearing)).
    ex = np.sin(bearing)
    ey = np.cos(bearing)
    nx = -np.cos(bearing)
    ny = np.sin(bearing)

    xy = np.asarray(xy, dtype=float)
    east = xy[..., 0] * ex + xy[..., 1] * nx
    north = xy[..., 0] * ey + xy[..., 1] * ny

    dlat = (north / EARTH_R) * (180.0 / math.pi)
    dlon = (east / (EARTH_R * math.cos(math.radians(anchor.lat0)))) * (180.0 / math.pi)
    lat = anchor.lat0 + dlat
    lon = anchor.lon0 + dlon
    return np.column_stack([lat, lon])


# -----------------------------------------------------------------------------
# Web Mercator tile math + Esri World Imagery fetcher
# -----------------------------------------------------------------------------

TILE_SIZE = 256
TILE_SOURCES = {
    'esri':   "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    'google': "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}",
    'google_hybrid': "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}",
    'osm':    "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
}
HEADERS = {"User-Agent": "H-infinity-rooftop-overlay/0.1 (+local research use)"}


def latlon_to_pixel(lat, lon, z):
    """Return (px, py) in global Web Mercator pixel space at zoom z."""
    n = 2.0 ** z
    px = (lon + 180.0) / 360.0 * n * TILE_SIZE
    lat_rad = math.radians(lat)
    py = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n * TILE_SIZE
    return px, py


def fetch_tile(z, x, y, cache_dir, source='google'):
    """Fetch one tile, cache to disk."""
    os.makedirs(cache_dir, exist_ok=True)
    fpath = os.path.join(cache_dir, f"{source}_{z}_{x}_{y}.jpg")
    if os.path.isfile(fpath):
        return Image.open(fpath).convert("RGB")
    url = TILE_SOURCES[source].format(z=z, x=x, y=y)
    r = requests.get(url, headers=HEADERS, timeout=20)
    r.raise_for_status()
    with open(fpath, "wb") as f:
        f.write(r.content)
    return Image.open(io.BytesIO(r.content)).convert("RGB")


def render_basemap(lat_min, lat_max, lon_min, lon_max, zoom, cache_dir,
                   source='google'):
    """Stitch tiles covering the lat/lon box at the given zoom; return
    (image, (px0, py0)) where (px0, py0) is the global pixel coord of the
    top-left of the stitched image.
    """
    px_a, py_a = latlon_to_pixel(lat_max, lon_min, zoom)   # NW corner
    px_b, py_b = latlon_to_pixel(lat_min, lon_max, zoom)   # SE corner
    tx_min = int(px_a // TILE_SIZE)
    tx_max = int(px_b // TILE_SIZE)
    ty_min = int(py_a // TILE_SIZE)
    ty_max = int(py_b // TILE_SIZE)

    w = (tx_max - tx_min + 1) * TILE_SIZE
    h = (ty_max - ty_min + 1) * TILE_SIZE
    canvas = Image.new("RGB", (w, h))
    for tx in range(tx_min, tx_max + 1):
        for ty in range(ty_min, ty_max + 1):
            tile = fetch_tile(zoom, tx, ty, cache_dir, source=source)
            canvas.paste(tile, ((tx - tx_min) * TILE_SIZE, (ty - ty_min) * TILE_SIZE))
    return canvas, (tx_min * TILE_SIZE, ty_min * TILE_SIZE)


def latlon_to_canvas(latlon, zoom, origin_px):
    px, py = latlon_to_pixel(latlon[0], latlon[1], zoom)
    return px - origin_px[0], py - origin_px[1]


# -----------------------------------------------------------------------------
# Top-level rendering
# -----------------------------------------------------------------------------

def make_overlay(family: str, family_kwargs: dict, out_png: str,
                 zoom: int = 20, padding_m: float = 6.0,
                 cache_dir: str = None, source: str = 'google'):
    if cache_dir is None:
        cache_dir = os.path.join(os.path.dirname(__file__), "tile_cache")

    anchor = Anchor(LAT0, LON0, BEARING_DEG)
    fn = PATH_FAMILIES[family]
    waypoints_local = fn(**family_kwargs)

    # Track rectangle corners in local frame, plus a padding box for the basemap.
    corners_local = np.array([
        [0.0, 0.0],
        [TRACK_LONG, 0.0],
        [TRACK_LONG, TRACK_SHORT],
        [0.0, TRACK_SHORT],
    ])
    # Padded box for tile coverage
    pad = padding_m
    bbox_local = np.array([
        [-pad, -pad],
        [TRACK_LONG + pad, -pad],
        [TRACK_LONG + pad, TRACK_SHORT + pad],
        [-pad, TRACK_SHORT + pad],
    ])

    waypoints_ll = local_to_latlon(waypoints_local, anchor)
    corners_ll = local_to_latlon(corners_local, anchor)
    bbox_ll = local_to_latlon(bbox_local, anchor)

    lat_min = float(bbox_ll[:, 0].min())
    lat_max = float(bbox_ll[:, 0].max())
    lon_min = float(bbox_ll[:, 1].min())
    lon_max = float(bbox_ll[:, 1].max())

    print(f"[render] source={source} zoom={zoom}  "
          f"lat=[{lat_min:.6f},{lat_max:.6f}]  "
          f"lon=[{lon_min:.6f},{lon_max:.6f}]")
    img, origin = render_basemap(lat_min, lat_max, lon_min, lon_max, zoom,
                                  cache_dir, source=source)

    # Convert lat/lon -> canvas pixels
    track_px = np.array([latlon_to_canvas(ll, zoom, origin) for ll in corners_ll])
    path_px = np.array([latlon_to_canvas(ll, zoom, origin) for ll in waypoints_ll])
    bbox_px = np.array([latlon_to_canvas(ll, zoom, origin) for ll in bbox_ll])

    # Trim canvas to padded bbox
    x0 = int(max(0, bbox_px[:, 0].min() - 4))
    y0 = int(max(0, bbox_px[:, 1].min() - 4))
    x1 = int(min(img.size[0], bbox_px[:, 0].max() + 4))
    y1 = int(min(img.size[1], bbox_px[:, 1].max() + 4))
    img = img.crop((x0, y0, x1, y1))
    track_px -= np.array([x0, y0])
    path_px -= np.array([x0, y0])

    # Plot
    fig, ax = plt.subplots(figsize=(12, 8), dpi=150)
    ax.imshow(np.asarray(img), interpolation='bilinear')
    ax.add_patch(MplPolygon(track_px, closed=True, fill=False,
                             edgecolor='yellow', linewidth=1.5,
                             linestyle='--', label='track 42x10 m'))
    ax.plot(path_px[:, 0], path_px[:, 1], '-', color='red',
            linewidth=2.0, label=f'path ({family})')
    ax.plot(path_px[0, 0], path_px[0, 1], 'o', color='lime',
            markersize=10, markeredgecolor='black', label='start')
    ax.plot(path_px[-1, 0], path_px[-1, 1], 's', color='red',
            markersize=10, markeredgecolor='black', label='end')

    # Compass: draw a north arrow in the upper-right
    ax.annotate('N', xy=(0.95, 0.95), xycoords='axes fraction',
                xytext=(0.95, 0.80), textcoords='axes fraction',
                ha='center', va='center', color='white', fontsize=14,
                arrowprops=dict(arrowstyle='->', color='white', lw=2))

    ax.set_axis_off()
    L_total = float(np.sum(np.linalg.norm(np.diff(waypoints_local, axis=0), axis=1)))
    ax.set_title(f"family={family}  L~={L_total:.1f} m  "
                 f"anchor=({LAT0:.6f},{LON0:.6f})  bearing={BEARING_DEG:g} deg")
    ax.legend(loc='lower right', framealpha=0.7)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f"[render] wrote {out_png}")

    # Sidecar GeoJSON for geojson.io / Google Earth
    geojson = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"name": "rooftop_track", "kind": "track_outline"},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[[ll[1], ll[0]] for ll in
                                      np.vstack([corners_ll, corners_ll[:1]])]],
                },
            },
            {
                "type": "Feature",
                "properties": {"name": f"path_{family}", "kind": "reference_path",
                               "length_m": L_total,
                               "params": family_kwargs},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[ll[1], ll[0]] for ll in waypoints_ll],
                },
            },
        ],
    }
    geojson_path = os.path.splitext(out_png)[0] + ".geojson"
    with open(geojson_path, "w") as f:
        json.dump(geojson, f, indent=2)
    print(f"[render] wrote {geojson_path}")

    return out_png, geojson_path


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def parse_kvs(items):
    out = {}
    for item in items or []:
        if '=' not in item:
            raise SystemExit(f"bad --param '{item}', expected k=v")
        k, v = item.split('=', 1)
        try:
            v_val = json.loads(v)
        except Exception:
            v_val = v
        out[k] = v_val
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--family', default='oval', choices=list(PATH_FAMILIES))
    ap.add_argument('--param', '-p', action='append',
                    help='family kwarg as k=v (JSON-parsed). Repeatable.')
    ap.add_argument('--zoom', type=int, default=20)
    ap.add_argument('--source', default='google',
                    choices=list(TILE_SOURCES))
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    kwargs = parse_kvs(args.param)
    out_png = args.out or os.path.join(os.path.dirname(__file__), 'out',
                                       f'overlay_{args.family}.png')
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    make_overlay(args.family, kwargs, out_png, zoom=args.zoom,
                 source=args.source)


if __name__ == '__main__':
    main()

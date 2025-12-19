#!/usr/bin/env python3
"""
Find longest-to-shortest (shortest-path) routes in a SLAM PGM map without specifying start/end.

Interpretation:
- Treat free space as a graph (4-connected).
- Compute long shortest paths (graph diameter-like paths) per connected component.
- Optionally collect Top-K longest shortest paths and reconstruct each with A*.

Outputs:
- Prints paths sorted by length (cells + meters if resolution given)
- Saves path overlays as PNGs

Usage:
  python3 longest_paths_from_pgm.py --pgm map.pgm --outdir out --topk 10 --max-cells 120000
Optional:
  --resolution 0.05   (meters/cell)
  --free-thresh 250   (pixel >= free_thresh is FREE)
  --occ-thresh 50     (pixel <= occ_thresh is OCCUPIED; between is UNKNOWN treated as blocked by default)
  --treat-unknown-free   (if you want UNKNOWN as traversable)
"""

from __future__ import annotations
import argparse
import os
import math
from collections import deque
import heapq
from typing import List, Tuple, Optional, Dict, Set

# -----------------------------
# Minimal PGM (P5/P2) reader
# -----------------------------
def read_pgm(path: str) -> Tuple[List[int], int, int, int]:
    """
    Returns: (pixels, width, height, maxval)
    pixels is a flat list length width*height, values 0..maxval
    Supports P2 (ASCII) and P5 (binary).
    """
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"Unsupported PGM magic: {magic!r}")

        def read_non_comment_line() -> bytes:
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            return line

        # width height
        wh = read_non_comment_line().split()
        while len(wh) < 2:
            wh += read_non_comment_line().split()
        w, h = int(wh[0]), int(wh[1])

        # maxval
        maxval_line = read_non_comment_line().strip()
        while not maxval_line:
            maxval_line = read_non_comment_line().strip()
        maxval = int(maxval_line)

        if magic == b"P2":
            # ASCII
            data = []
            while len(data) < w * h:
                line = read_non_comment_line()
                data.extend([int(x) for x in line.split()])
            data = data[: w * h]
            return data, w, h, maxval

        # P5 binary
        # Skip single whitespace after maxval if present
        ch = f.read(1)
        if ch not in (b"\n", b"\r", b" ", b"\t"):
            # not whitespace; put it back
            f.seek(-1, 1)

        # Read raster
        if maxval < 256:
            raw = f.read(w * h)
            if len(raw) != w * h:
                raise ValueError("Unexpected EOF reading PGM raster")
            data = list(raw)
        else:
            # 2 bytes per pixel (big endian)
            raw = f.read(w * h * 2)
            if len(raw) != w * h * 2:
                raise ValueError("Unexpected EOF reading PGM raster")
            data = []
            for i in range(0, len(raw), 2):
                data.append((raw[i] << 8) + raw[i + 1])
        return data, w, h, maxval


# -----------------------------
# Grid / graph helpers
# -----------------------------
def idx(x: int, y: int, w: int) -> int:
    return y * w + x

def xy(i: int, w: int) -> Tuple[int, int]:
    return (i % w, i // w)

def nhood4(i: int, w: int, h: int) -> List[int]:
    x, y = xy(i, w)
    out = []
    if x > 0: out.append(idx(x - 1, y, w))
    if x + 1 < w: out.append(idx(x + 1, y, w))
    if y > 0: out.append(idx(x, y - 1, w))
    if y + 1 < h: out.append(idx(x, y + 1, w))
    return out

def manhattan(a: int, b: int, w: int) -> int:
    ax, ay = xy(a, w)
    bx, by = xy(b, w)
    return abs(ax - bx) + abs(ay - by)

def reconstruct(came_from: Dict[int, int], goal: int) -> List[int]:
    path = [goal]
    while path[-1] in came_from:
        path.append(came_from[path[-1]])
    path.reverse()
    return path

def astar(start: int, goal: int, free: List[bool], w: int, h: int) -> Optional[List[int]]:
    """
    A* on 4-connected uniform-cost grid.
    Returns list of indices path (including start and goal) or None.
    """
    if start == goal:
        return [start]
    if not free[start] or not free[goal]:
        return None

    open_heap = []
    heapq.heappush(open_heap, (manhattan(start, goal, w), 0, start))
    came_from: Dict[int, int] = {}
    gscore: Dict[int, int] = {start: 0}
    closed: Set[int] = set()

    while open_heap:
        f, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            return reconstruct(came_from, goal)
        closed.add(cur)

        for nb in nhood4(cur, w, h):
            if not free[nb]:
                continue
            ng = g + 1
            if nb not in gscore or ng < gscore[nb]:
                gscore[nb] = ng
                came_from[nb] = cur
                nf = ng + manhattan(nb, goal, w)
                heapq.heappush(open_heap, (nf, ng, nb))
    return None

def bfs_distances(start: int, free: List[bool], w: int, h: int) -> Tuple[List[int], int]:
    """
    BFS shortest-path distances from start over free cells.
    Returns (dist array, farthest_index).
    dist = -1 for unreachable.
    """
    dist = [-1] * (w * h)
    q = deque()
    dist[start] = 0
    q.append(start)
    far = start

    while q:
        cur = q.popleft()
        if dist[cur] > dist[far]:
            far = cur
        for nb in nhood4(cur, w, h):
            if free[nb] and dist[nb] == -1:
                dist[nb] = dist[cur] + 1
                q.append(nb)
    return dist, far

def connected_components(free: List[bool], w: int, h: int) -> List[List[int]]:
    """
    Return list of components, each as list of cell indices.
    """
    seen = [False] * (w * h)
    comps = []
    for i in range(w * h):
        if free[i] and not seen[i]:
            q = deque([i])
            seen[i] = True
            comp = [i]
            while q:
                cur = q.popleft()
                for nb in nhood4(cur, w, h):
                    if free[nb] and not seen[nb]:
                        seen[nb] = True
                        q.append(nb)
                        comp.append(nb)
            comps.append(comp)
    return comps


# -----------------------------
# Visualization (no pillow needed)
# Writes simple PPM and converts to PNG if pillow exists (optional).
# We'll just write PPM; most viewers can open it, or you can convert with ImageMagick.
# -----------------------------
def write_ppm_overlay(outpath: str, pixels: List[int], w: int, h: int,
                      path: List[int],
                      free_thresh: int, occ_thresh: int):
    """
    Make an RGB image:
    - occupied: dark
    - free: light
    - unknown: mid
    - path: red
    Writes PPM (P3) ASCII.
    """
    path_set = set(path)

    def base_color(v: int) -> Tuple[int, int, int]:
        if v <= occ_thresh:
            return (40, 40, 40)
        if v >= free_thresh:
            return (230, 230, 230)
        return (140, 140, 140)  # unknown

    with open(outpath, "w") as f:
        f.write("P3\n")
        f.write(f"{w} {h}\n255\n")
        for y in range(h):
            for x in range(w):
                i = idx(x, y, w)
                r, g, b = base_color(pixels[i])
                if i in path_set:
                    r, g, b = (255, 0, 0)
                f.write(f"{r} {g} {b} ")
            f.write("\n")


# -----------------------------
# Main: choose endpoints automatically and rank paths
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", default='/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/maps/map1/map_1766011303.pgm', help="SLAM-generated map image in .pgm")
    ap.add_argument("--outdir", default="frontier_paths_out", help="Output directory")
    ap.add_argument("--topk", type=int, default=5, help="How many longest shortest paths to output")
    ap.add_argument("--max-cells", type=int, default=150000,
                    help="Safety limit: if free cells exceed this, Top-K search becomes expensive.")
    ap.add_argument("--resolution", type=float, default=0.0, help="meters per cell (optional)")
    ap.add_argument("--free-thresh", type=int, default=250, help="pixel >= free_thresh => free")
    ap.add_argument("--occ-thresh", type=int, default=50, help="pixel <= occ_thresh => occupied")
    ap.add_argument("--treat-unknown-free", action="store_true",
                    help="Treat unknown (between thresholds) as traversable")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    pixels, w, h, maxval = read_pgm(args.pgm)
    if maxval != 255:
        # Normalize to 0..255 for thresholds
        scale = 255.0 / float(maxval)
        pixels = [int(round(v * scale)) for v in pixels]

    # Build free mask
    free = [False] * (w * h)
    for i, v in enumerate(pixels):
        if v >= args.free_thresh:
            free[i] = True
        elif args.treat_unknown_free and (args.occ_thresh < v < args.free_thresh):
            free[i] = True

    free_cells = sum(1 for x in free if x)
    print(f"[INFO] Map loaded: {w}x{h}, free_cells={free_cells}")

    if free_cells == 0:
        print("[ERROR] No free cells found. Adjust thresholds (--free-thresh / --occ-thresh).")
        return

    comps = connected_components(free, w, h)
    comps.sort(key=len, reverse=True)
    print(f"[INFO] Found {len(comps)} free-space connected components.")
    for i, c in enumerate(comps[:5]):
        print(f"  component[{i}] size={len(c)}")

    # If huge, we still can compute one diameter per component, but Top-K over all pairs is costly.
    if free_cells > args.max_cells:
        print(f"[WARN] free_cells={free_cells} exceeds max-cells={args.max_cells}. "
              f"Will compute ONE longest path per component (diameter-like), not Top-K across all pairs.")
        topk = min(args.topk, len(comps))
        results = []
        for ci in range(topk):
            comp = comps[ci]
            seed = comp[0]
            _, a = bfs_distances(seed, free, w, h)
            dist_a, b = bfs_distances(a, free, w, h)
            length = dist_a[b]
            path = astar(a, b, free, w, h)
            results.append((length, a, b, path))
        results.sort(key=lambda x: x[0], reverse=True)
    else:
        # Top-K longest shortest paths (approx but good):
        # For each component:
        #   1) pick multiple seeds (subset) -> find farthest pairs
        # This avoids O(V^2). Still can be heavy, but manageable for moderate maps.
        seeds_per_comp = 10 if free_cells < 60000 else 5
        candidate_pairs = []  # (dist, a, b)

        for comp in comps:
            if len(comp) < 50:
                continue
            step = max(1, len(comp) // seeds_per_comp)
            seeds = [comp[i] for i in range(0, len(comp), step)][:seeds_per_comp]

            # For each seed: BFS -> farthest a, BFS(a) -> farthest b
            for s in seeds:
                _, a = bfs_distances(s, free, w, h)
                dist_a, b = bfs_distances(a, free, w, h)
                d = dist_a[b]
                if d >= 0:
                    candidate_pairs.append((d, a, b))

        # Deduplicate (a,b) vs (b,a)
        uniq = {}
        for d, a, b in candidate_pairs:
            key = (a, b) if a < b else (b, a)
            if key not in uniq or d > uniq[key][0]:
                uniq[key] = (d, a, b)

        candidates = list(uniq.values())
        candidates.sort(key=lambda x: x[0], reverse=True)

        results = []
        for d, a, b in candidates:
            if len(results) >= args.topk:
                break
            path = astar(a, b, free, w, h)
            if path is None:
                continue
            results.append((d, a, b, path))

        results.sort(key=lambda x: x[0], reverse=True)

    if not results:
        print("[WARN] No paths found. This can happen if free space is tiny or thresholds are wrong.")
        return

    # Output
    print("\n[RESULTS] Longest -> shortest (shortest-path length in cells):")
    for rank, (d, a, b, path) in enumerate(results, 1):
        ax, ay = xy(a, w)
        bx, by = xy(b, w)
        meters = (d * args.resolution) if args.resolution > 0 else None
        if meters is not None:
            print(f"{rank:02d}) len={d} cells ({meters:.2f} m)  A=({ax},{ay})  B=({bx},{by})")
        else:
            print(f"{rank:02d}) len={d} cells  A=({ax},{ay})  B=({bx},{by})")

        # Save overlay
        ppm_path = os.path.join(args.outdir, f"path_{rank:02d}_len_{d}.ppm")
        write_ppm_overlay(ppm_path, pixels, w, h, path, args.free_thresh, args.occ_thresh)
        print(f"     saved overlay: {ppm_path}")

    print("\nDone. Tip: open .ppm with an image viewer or convert:")
    print("  convert path_01_*.ppm path_01.png   (ImageMagick)")
    print("or:")
    print("  ffmpeg -i path_01_*.ppm path_01.png")

if __name__ == "__main__":
    main()

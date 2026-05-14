#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


PGM_OCCUPIED = 0
PGM_UNKNOWN = 205
PGM_FREE = 254


def parse_header(path: Path) -> Tuple[Dict[str, str], int]:
    header: Dict[str, str] = {}
    offset = 0
    with path.open("rb") as file:
        while True:
            line = file.readline()
            if not line:
                raise ValueError("PCD header ended before DATA line")
            offset += len(line)
            text = line.decode("ascii", errors="replace").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split(maxsplit=1)
            key = parts[0].upper()
            value = parts[1] if len(parts) > 1 else ""
            header[key] = value
            if key == "DATA":
                break
    return header, offset


def pcd_numpy_type(size: int, field_type: str):
    field_type = field_type.upper()
    if field_type == "F":
        if size == 4:
            return np.float32
        if size == 8:
            return np.float64
    if field_type == "I":
        return {1: np.int8, 2: np.int16, 4: np.int32, 8: np.int64}[size]
    if field_type == "U":
        return {1: np.uint8, 2: np.uint16, 4: np.uint32, 8: np.uint64}[size]
    raise ValueError(f"Unsupported PCD field type: TYPE={field_type} SIZE={size}")


def pcd_dtype(header: Dict[str, str]) -> np.dtype:
    fields = header["FIELDS"].split()
    sizes = [int(value) for value in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(value) for value in header.get("COUNT", " ".join(["1"] * len(fields))).split()]

    dtype_fields = []
    for name, size, field_type, count in zip(fields, sizes, types, counts):
        base_type = pcd_numpy_type(size, field_type)
        if count == 1:
            dtype_fields.append((name, base_type))
        else:
            dtype_fields.append((name, base_type, (count,)))
    return np.dtype(dtype_fields)


def read_points(path: Path) -> np.ndarray:
    header, data_offset = parse_header(path)
    data_mode = header["DATA"].lower()
    fields = header["FIELDS"].split()
    for required in ("x", "y", "z"):
        if required not in fields:
            raise ValueError(f"PCD file is missing required '{required}' field")

    points = int(header.get("POINTS", "0") or 0)
    if points <= 0:
        width = int(header.get("WIDTH", "0") or 0)
        height = int(header.get("HEIGHT", "1") or 1)
        points = width * height

    if data_mode == "binary":
        dtype = pcd_dtype(header)
        raw = np.fromfile(path, dtype=dtype, count=points, offset=data_offset)
        xyz = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float64, copy=False)
        return xyz

    if data_mode == "ascii":
        data = np.loadtxt(path, comments="#", skiprows=count_header_lines(path))
        if data.ndim == 1:
            data = data.reshape(1, -1)
        indices = [fields.index(name) for name in ("x", "y", "z")]
        return data[:, indices].astype(np.float64, copy=False)

    if data_mode == "binary_compressed":
        raise ValueError("binary_compressed PCD is not supported by this lightweight converter")

    raise ValueError(f"Unsupported PCD DATA mode: {header['DATA']}")


def count_header_lines(path: Path) -> int:
    count = 0
    with path.open("rb") as file:
        for line in file:
            count += 1
            if line.decode("ascii", errors="replace").strip().upper().startswith("DATA"):
                break
    return count


def build_grid(
    points: np.ndarray,
    resolution: float,
    padding: float,
    z_min: float,
    z_max: float,
    unknown_background: bool,
    occupied_dilation: int,
) -> Tuple[np.ndarray, Tuple[float, float, float]]:
    finite_mask = np.isfinite(points).all(axis=1)
    height_mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    selected = points[finite_mask & height_mask]
    if selected.size == 0:
        raise ValueError(
            "No points survived filtering. Relax --z-min/--z-max or check the PCD coordinate frame."
        )

    min_x = math.floor((float(np.min(selected[:, 0])) - padding) / resolution) * resolution
    min_y = math.floor((float(np.min(selected[:, 1])) - padding) / resolution) * resolution
    max_x = math.ceil((float(np.max(selected[:, 0])) + padding) / resolution) * resolution
    max_y = math.ceil((float(np.max(selected[:, 1])) + padding) / resolution) * resolution

    width = int(math.ceil((max_x - min_x) / resolution)) + 1
    height = int(math.ceil((max_y - min_y) / resolution)) + 1

    background = PGM_UNKNOWN if unknown_background else PGM_FREE
    grid = np.full((height, width), background, dtype=np.uint8)

    cols = np.floor((selected[:, 0] - min_x) / resolution).astype(np.int64)
    rows = np.floor((selected[:, 1] - min_y) / resolution).astype(np.int64)
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    grid[rows[valid], cols[valid]] = PGM_OCCUPIED

    if occupied_dilation > 0:
        occupied = np.argwhere(grid == PGM_OCCUPIED)
        for row, col in occupied:
            r0 = max(0, row - occupied_dilation)
            r1 = min(height, row + occupied_dilation + 1)
            c0 = max(0, col - occupied_dilation)
            c1 = min(width, col + occupied_dilation + 1)
            grid[r0:r1, c0:c1] = PGM_OCCUPIED

    # PGM origin is top-left, Nav2 map origin is bottom-left.
    grid = np.flipud(grid)
    return grid, (min_x, min_y, 0.0)


def write_pgm(path: Path, grid: np.ndarray) -> None:
    with path.open("wb") as file:
        file.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode("ascii"))
        file.write(grid.tobytes())


def write_yaml(path: Path, image_name: str, resolution: float, origin: Tuple[float, float, float]) -> None:
    content = (
        f"image: {image_name}\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    path.write_text(content, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Project a LIO-SAM PCD map into a Nav2-compatible PGM/YAML occupancy map."
    )
    parser.add_argument("--pcd", required=True, type=Path, help="Input PCD file, usually GlobalMap.pcd.")
    parser.add_argument(
        "--output-prefix",
        required=True,
        type=Path,
        help="Output path without extension, e.g. /home/user/teb_ws/src/teb_nav2_demo/maps/liosam_map.",
    )
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters per cell.")
    parser.add_argument("--padding", type=float, default=1.0, help="Free/unknown border around the point cloud.")
    parser.add_argument(
        "--z-min",
        type=float,
        default=0.15,
        help="Minimum z height to project as obstacles. Raise this to remove ground points.",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=2.0,
        help="Maximum z height to project as obstacles.",
    )
    parser.add_argument(
        "--unknown-background",
        action="store_true",
        help="Use unknown background instead of free background.",
    )
    parser.add_argument(
        "--occupied-dilation",
        type=int,
        default=0,
        help="Expand occupied cells by this many grid cells before writing the map.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.resolution <= 0:
        raise ValueError("--resolution must be positive")
    if args.z_min > args.z_max:
        raise ValueError("--z-min cannot be greater than --z-max")

    pcd_path = args.pcd.expanduser().resolve()
    output_prefix = args.output_prefix.expanduser().resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    points = read_points(pcd_path)
    grid, origin = build_grid(
        points=points,
        resolution=args.resolution,
        padding=args.padding,
        z_min=args.z_min,
        z_max=args.z_max,
        unknown_background=args.unknown_background,
        occupied_dilation=max(0, args.occupied_dilation),
    )

    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, pgm_path.name, args.resolution, origin)

    occupied_count = int(np.count_nonzero(grid == PGM_OCCUPIED))
    print(f"Input PCD:     {pcd_path}")
    print(f"Output PGM:    {pgm_path}")
    print(f"Output YAML:   {yaml_path}")
    print(f"Grid:          {grid.shape[1]} x {grid.shape[0]} cells")
    print(f"Resolution:    {args.resolution:.3f} m/cell")
    print(f"Origin:        [{origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f}]")
    print(f"Occupied:      {occupied_count} cells")


if __name__ == "__main__":
    main()

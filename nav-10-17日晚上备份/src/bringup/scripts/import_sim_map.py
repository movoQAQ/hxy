#!/usr/bin/env python3
"""Align a normalized binary XYZ PCD to the RMUC simulation and project its map.

Requires numpy, opencv-python and PyYAML. The input PCD must be a surface sample
of the simulation STL translated so that its minimum XYZ is zero. Assets are
written beside the installed bringup map configuration, never to the source PCD.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    with args.pcd.open('rb') as stream:
        header = {}
        while True:
            line = stream.readline().decode('ascii').strip()
            if not line:
                raise ValueError('Unexpected end of PCD header')
            if line.startswith('#'):
                continue
            key, *value = line.split()
            header[key] = value
            if key == 'DATA':
                break
        if header['DATA'] != ['binary'] or any(x != '1' for x in header['COUNT']):
            raise ValueError('Expected binary PCD with scalar fields')
        dtype = np.dtype([(name, '<' + {'F': 'f', 'I': 'i', 'U': 'u'}[kind] + size)
                          for name, kind, size in zip(header['FIELDS'], header['TYPE'], header['SIZE'])])
        raw = np.frombuffer(stream.read(), dtype=dtype)
    xyz = np.column_stack([raw[name] for name in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    mesh_bytes = args.mesh.read_bytes()
    triangle_dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.frombuffer(mesh_bytes, dtype=triangle_dtype, offset=84)
    vertices = triangles['vertices'].reshape(-1, 3)
    if not np.allclose(np.ptp(xyz, axis=0), np.ptp(vertices, axis=0), atol=0.02):
        raise ValueError('PCD and STL bounds differ; check the map model and scale')
    world = ET.parse(args.world).getroot().find('world')
    model = world.find("model[@name='rmuc_2027']")
    pose = np.fromstring(model.findtext('pose'), sep=' ')
    if not np.allclose(pose[3:], 0):
        raise ValueError('Expected an untranslated-axis mesh (zero RPY)')
    offset = vertices.min(axis=0) + pose[:3]
    xyz += offset

    resolution = 0.05
    floor_z = 0.2
    origin = np.floor((xyz[:, :2].min(axis=0) - 0.5) / resolution) * resolution
    width, height = np.ceil((xyz[:, :2].max(axis=0) + 0.5 - origin) / resolution).astype(int) + 1
    floor = np.zeros((height, width), np.uint8)
    obstacles = np.zeros_like(floor)
    cells = np.floor((xyz[:, :2] - origin) / resolution).astype(int)
    ground = (xyz[:, 2] >= floor_z - 0.08) & (xyz[:, 2] <= floor_z + 0.08)
    blocked = (xyz[:, 2] > floor_z + 0.12) & (xyz[:, 2] <= floor_z + 1.2)
    floor[cells[ground, 1], cells[ground, 0]] = 255
    obstacles[cells[blocked, 1], cells[blocked, 0]] = 255
    # Close gaps between sampled surface points without opening obstacle cells.
    kernel = np.ones((3, 3), np.uint8)
    floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, kernel)
    obstacles = cv2.morphologyEx(obstacles, cv2.MORPH_CLOSE, kernel)
    # The simulation adds perimeter boxes that are not in the supplied mesh PCD.
    walls = world.find("model[@name='rmuc_2027_perimeter_walls']")
    if walls is not None:
        wall_points = []
        for link in walls.findall('link'):
            center = np.fromstring(link.findtext('pose'), sep=' ')[:3]
            size = np.fromstring(link.findtext('collision/geometry/box/size'), sep=' ')
            low, high = center - size / 2, center + size / 2
            a, b = np.floor((low[:2] - origin) / resolution).astype(int), np.ceil((high[:2] - origin) / resolution).astype(int)
            cv2.rectangle(obstacles, tuple(a), tuple(b), 255, -1)
            axis = int(np.argmax(size[:2]))
            along = np.arange(low[axis], high[axis] + resolution, resolution)
            z = np.arange(max(low[2], -0.1), min(high[2], 2.0), resolution)
            aa, zz = np.meshgrid(along, z)
            points = np.full((aa.size, 3), 0.0, dtype=np.float32)
            points[:, axis] = aa.ravel(); points[:, 2] = zz.ravel()
            points[:, 1 - axis] = center[1 - axis]
            wall_points.append(points)
        xyz = np.vstack([xyz, *wall_points])
    occupancy = np.full_like(floor, 205)
    occupancy[floor > 0] = 254
    occupancy[obstacles > 0] = 0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output_dir / 'rmuc_2027.pgm'), np.flipud(occupancy))
    config = dict(image='rmuc_2027.pgm', resolution=resolution,
                  origin=[float(origin[0]), float(origin[1]), 0.0],
                  negate=0, occupied_thresh=0.65, free_thresh=0.196)
    (args.output_dir / 'rmuc_2027.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    with (args.output_dir / 'rmuc_2027.pcd').open('wb') as stream:
        stream.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                      f'COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                      f'POINTS {len(xyz)}\nDATA binary\n').encode())
        stream.write(np.asarray(xyz, dtype='<f4').tobytes())
    metadata = dict(source_pcd=str(args.pcd), source_sha256=hashlib.sha256(args.pcd.read_bytes()).hexdigest(),
                    source_mesh=str(args.mesh), pcd_to_world_translation=offset.tolist(),
                    frame='map (Gazebo world)', floor_z=floor_z,
                    obstacle_height_band=[floor_z + 0.12, floor_z + 1.2],
                    points=len(xyz), map_size=[int(width), int(height)])
    (args.output_dir / 'rmuc_2027.metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()

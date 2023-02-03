'''
fieldmap [red]
    generates from red origin
by default generates from blue origin
'''

import json
import sys
from math import sin, cos, pi

def deg_to_rad(deg: float) -> float:
    return deg * pi / 180.0

def deadband(val: float) -> float:
    if abs(val) < 1e-10:
        return 0
    return val

FIELD_WIDTH = 16.54
FIELD_HEIGHT = 8.02
# apriltag poses in meters from blue origin
POSES = [
    (15.513558, 1.071626, 0.462788, 180),
    (15.513558, 2.748026, 0.462788, 180),
    (15.513558, 4.424426, 0.462788, 180),
    (16.178784, 6.749796, 0.695452, 180),
    
    (0.36195, 6.749796, 0.695452, 0),
    (1.02743, 4.424426, 0.462788, 0),
    (1.02743, 2.748026, 0.462788, 0),
    (1.02743, 1.071626, 0.462788, 0),
]

def red():
    global POSES

    for i in range(len(POSES)):
        POSES[i] = (
            round(FIELD_WIDTH - POSES[i][0], 6),
            round(FIELD_HEIGHT - POSES[i][1], 6),
            POSES[i][2],
            (POSES[i][3] + 180) % 360
        )

def get_transform(x: float, y: float, z: float, t: float) -> list:
    t = deg_to_rad(t)
    return [
        deadband(cos(t)), deadband(-sin(t)), 0, x,
        deadband(sin(t)), deadband( cos(t)), 0, y,
        0, 0, 1, z,
        0, 0, 0, 1
    ]

def get_tag(id: int, transform: list) -> dict:
    return {
        "family": "apriltag3_16h5_classic",
        "id": id,
        "size": 152.4,
        "transform": transform,
        "unique": 1
    }


if len(sys.argv) > 1:
    if sys.argv[1][:3] == 'red':
        red()

with open('fieldmap.fmap', 'w') as f:
    f.write(json.dumps({
        'fiducials': [
            get_tag(i + 1, get_transform(*p))
                for i, p in enumerate(POSES)
        ]
    }, indent=4))
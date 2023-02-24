'''
arguments:
    path name
    start point type
    (mid point types)
    end point type
point types:
    peg<0-5>
    cube<0-2>
    intake<0-4>
'''

import json
import sys

def inches_to_meters(inches: float) -> float:
    return inches * 0.0254

# including bumpers
ROBOT_WIDTH = inches_to_meters(31)
GRID_DEPTH = inches_to_meters(54.25)

args = len(sys.argv) - 1

if args < 3:
    print('Usage: generate_path_from_score_spots path_name start_point [mid_point...] end_point')
    print('Point types:')
    print('  peg<0-5>')
    print('  cube<0-2>')
    print('ex. generate_path_from_score_spots "Four Piece" peg0 cube0 cube1 cube3')
    exit(1)

peg_y_values = list(map(inches_to_meters, [
    196.0,
    152.0,
    130.0,
    86.0,
    64.0,
    20.0
]))

cube_y_values = list(map(inches_to_meters, [
    174.125,
    108.125,
    42.125
]))

def get_point(point_type: str) -> tuple:
    if point_type.startswith('peg'):
        return (
            GRID_DEPTH + ROBOT_WIDTH / 2,
            peg_y_values[int(point_type[3])])
    elif point_type.startswith('cube'):
        return (
            GRID_DEPTH + ROBOT_WIDTH / 2,
            cube_y_values[int(point_type[4])])
    else:
        print(f'Invalid point type: {point_type}')
        exit(1)

def create_waypoint(anchor: tuple, prev: bool, next: bool) -> dict:
    return {
        'anchorPoint': {
            'x': anchor[0],
            'y': anchor[1]
        },
        'prevControl': ({ 'x': anchor[0] - 1, 'y': anchor[1] } if prev  else None),
        'nextControl': ({ 'x': anchor[0] + 1, 'y': anchor[1] } if next  else None),
        'holonomicAngle': 180,
        'isReversal': False,
        'velOverride': None,
        'isLocked': False,
        'isStopPoint': False,
        'stopEvent': {
            'names': [],
            'executionBehavior': 'parallel',
            'waitBehavior': 'none',
            'waitTime': 0
        }
    }

waypoints = []

for i in range(args - 1):
    p = get_point(sys.argv[i + 2])
    # convert waypoint to json string, add to string
    waypoints.append(create_waypoint(
        p,
        i != 0,
        i != args - 2))

with open('../src/main/deploy/pathplanner/' + sys.argv[1] + '.path', 'w') as f:
    f.write(json.dumps({
        'waypoints': waypoints,
        'markers': []
    }, indent=2))

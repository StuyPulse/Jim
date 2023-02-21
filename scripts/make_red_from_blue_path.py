import json
import sys

FIELD_WIDTH = 16.54

def transform(coord: dict) -> dict:
    if coord is None:
        return None

    return {
        'x': FIELD_WIDTH - coord['x'],
        'y': coord['y']
    }


if len(sys.argv) < 3:
    print('Usage: make_red_from_blue_path input_path output_name')
    exit(1)

with open(sys.argv[1], 'r') as r:
    data = json.load(r)

for i in range(len(data['waypoints'])):
    w = data['waypoints'][i]

    w['anchorPoint'] = transform(w['anchorPoint'])
    w['prevControl'] = transform(w['prevControl'])
    w['nextControl'] = transform(w['nextControl'])
    w['holonomicAngle'] = 180 - w['holonomicAngle']

with open('../src/main/deploy/pathplanner/' + sys.argv[2] + '.path', 'w') as f:
    json.dump(data, f, indent=2)

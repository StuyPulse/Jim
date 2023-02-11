def parse_line(line):
    args = line.split(',')
    
    timestamp = float(args[0])
    name = args[1][20:-1]
    value = float(args[2])
    
    return timestamp, name, value

total_values_x = 0
current_sum_x = 0

total_values_y = 0
current_sum_y = 0

total_values_r = 0
current_sum_r = 0

odometry_x_values = []
odometry_y_values = []
odometry_r_values = []

with open('csv/StandardDeviationsLow.csv', 'r') as f:
    for l in f.readlines()[1:]:
        timestamp, name, value = parse_line(l)
        
        if (name.startswith('Vision/limelight')):
            if ('X' in name):
                current_sum_x += (odometry_x_values[-1] - value)**2
                total_values_x += 1
            if ('Y' in name):
                current_sum_y += (odometry_y_values[-1] - value)**2
                total_values_y += 1
            if ('Rotation' in name):
                current_sum_r += (odometry_r_values[-1] - value)**2
                total_values_r += 1
        
        if (name.startswith('Odometry/Pose Estimator')):
            if ('X' in name):
                odometry_x_values.append(value)
            if ('Y' in name):
                odometry_y_values.append(value)
            if ('Rotation' in name):
                odometry_r_values.append(value)
    print('X: ', (current_sum_x / total_values_x)**0.5)
    print('Y: ', (current_sum_y / total_values_y)**0.5)
    print('R: ', (current_sum_r / total_values_r)**0.5)
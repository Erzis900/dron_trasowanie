import turtle
import math
from shapely.geometry import Point, Polygon
import time
from scipy.spatial import ConvexHull

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
    
def get_angles(poly_coord):
    lines = []
    angles = []

    for i in range(len(poly_coord) - 1):
        line = [poly_coord[i], poly_coord[i + 1]]
        lines.append(line)

    line = [poly_coord[len(poly_coord) - 1], poly_coord[0]]
    lines.append(line)

    sum_angles = 0

    for i in range(len(lines) - 1):
        angles.append(get_angle(lines[i], lines[i + 1]))
        sum_angles += get_angle(lines[i], lines[i + 1])

    angles.append(get_angle(lines[0], lines[len(lines) - 1]))
    sum_angles += get_angle(lines[0], lines[len(lines) - 1])

    print("\nSuma katow: " + str(sum_angles))

    return angles

def draw_polygon(poly_coord, color):
    t.color(color)

    t.up()
    t.goto(poly_coord[0])
    t.down()

    for i in range(1, len(poly_coord)):
        t.goto(poly_coord[i])

    t.goto(poly_coord[0])

def draw_rectangle(x, y, w, h, color, angle = 0):
    t.color(color)
    t.up()
    t.goto(x, y)
    t.seth(angle)
    t.down()

    for i in range(2):
        t.forward(w)
        t.left(90)
        t.forward(h)
        t.left(90)

def rotate_point(point, origin, angle):
    ox, oy = origin
    px, py = point
    
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    
    return qx, qy

def get_waypoints(poly_coord, rect_coord, fov, pen_color, should_draw, rect_dim, starting_wp, angle = 0):
    rows = math.floor(rect_dim[0] / fov[0])
    cols = math.floor(rect_dim[1] / fov[1])

    turtle.tracer(0)

    x = rect_coord[0][0]
    y = rect_coord[0][1]

    waypoints = []

    rot_rect_x_start, rot_rect_y_start = None, None

    for i in range(cols):
        for j in range(rows):
            rect_x = x + j * fov[0]
            rect_y = y + i * fov[1]

            rot_rect_x, rot_rect_y = rotate_point((rect_x, rect_y), (x, y), math.radians(angle))

            rot_rect_x = round(rot_rect_x, 2)
            rot_rect_y = round(rot_rect_y, 2)

            w = rotate_point((fov[0], 0), (0, 0), math.radians(angle))
            h = rotate_point((0, fov[1]), (0, 0), math.radians(angle))

            center_x = rot_rect_x + w[0] / 2 + h[0] / 2
            center_y = rot_rect_y + w[1] / 2 + h[1] / 2

            waypoint = (center_x, center_y)

            if(is_inside(poly_coord, center_x, center_y)):
                if waypoint not in waypoints:
                    waypoints.append(waypoint)

                    if should_draw:
                        if math.isclose(waypoint[0], starting_wp[0]) and math.isclose(waypoint[1], starting_wp[1]):
                            rot_rect_x_start = rot_rect_x
                            rot_rect_y_start = rot_rect_y
                        else:
                            draw_rectangle(rot_rect_x, rot_rect_y, fov[0], fov[1], pen_color, angle)

    if should_draw and rot_rect_x_start != None and rot_rect_y_start != None:
        draw_rectangle(rot_rect_x_start, rot_rect_y_start, fov[0], fov[1], "blue", angle)

    should_draw = False
    return waypoints

def get_angle(line_1, line_2):
    x1, y1 = line_1[0]
    x2, y2 = line_1[1]
    x3, y3 = line_2[0]
    x4, y4 = line_2[1]

    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3

    magnitude_1 = math.sqrt((dx1)**2 + (dy1)**2)
    magnitude_2 = math.sqrt((dx2)**2 + (dy2)**2)

    product = dx1 * dx2 + dy1 * dy2

    cos_angle = product / (magnitude_1 * magnitude_2)

    angle = math.degrees(math.acos(cos_angle))

    if dx1 != 0 and dx2 != 0:
        slope_1 = dy1 / dx1
        slope_2 = dy2 / dx2

        if slope_1 > slope_2:
            if line_1[1] == line_2[1] and slope_1 * slope_2 > 0:
                angle = 360 - angle 

    if line_1[1] != line_2[1] and line_1[0] != line_2[0]:
        angle = 180 - angle

    return angle

def get_rect(poly_coord):
    n_sides = len(poly_coord)

    min_x = min(coord[0] for coord in poly_coord)
    max_x = max(coord[0] for coord in poly_coord)
    min_y = min(coord[1] for coord in poly_coord)
    max_y = max(coord[1] for coord in poly_coord)

    w = max_x - min_x
    h = max_y - min_y

    centroid = (sum([p[0] for p in poly_coord]) / n_sides, sum([p[1] for p in poly_coord]) / n_sides)
    
    center_x = centroid[0]
    center_y = centroid[1]

    if w > h:
        size_x = size_y = w
    else:
        size_x = size_y = h

    rect_coord = [(center_x - size_x, center_y - size_y), (center_x + size_x, center_y - size_y), (center_x + size_x, center_y + size_y), (center_x - size_x, center_y + size_y)]

    return rect_coord

def rotate_polygon(poly_coord, angle):
    n_sides = len(poly_coord)

    centroid = (sum([p[0] for p in poly_coord]) / n_sides, sum([p[1] for p in poly_coord]) / n_sides)
    poly_coord = [(p[0] - centroid[0], p[1] - centroid[1]) for p in poly_coord]

    angle_radians = math.radians(angle)

    rotation_matrix = [[math.cos(angle_radians), -math.sin(angle_radians)],
                        [math.sin(angle_radians), math.cos(angle_radians)]]    
    
    rotated_poly = []
    for p in poly_coord:
        x_new = rotation_matrix[0][0] * p[0] + rotation_matrix[0][1] * p[1]
        y_new = rotation_matrix[1][0] * p[0] + rotation_matrix[1][1] * p[1]
        rotated_poly.append((x_new, y_new))
    
    rotated_poly = [(p[0] + centroid[0], p[1] + centroid[1]) for p in rotated_poly]

    return rotated_poly

def is_inside(poly_coord, waypoint_x, waypoint_y):
    polygon = Polygon(poly_coord)
    point = Point(waypoint_x, waypoint_y)

    return polygon.contains(point)

def draw_path(points):
    t.color("red")
    t.up()
    x, y = points[0]
    t.goto(x, y)
    t.down()

    for x, y in points:
        t.goto(x, y)

def euclidean_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def find_path(waypoints, start):
    waypoints.sort(key = lambda x: (x[1], x[0]))
    
    reverse_rows = start[1] > (waypoints[0][1] + waypoints[-1][1]) / 2

    rows = []
    current_row = []
    for i, waypoint in enumerate(waypoints):
        if i == 0 or waypoint[1] == previous_waypoint[1]:
            current_row.append(waypoint)
        else:
            rows.append(current_row)
            current_row = [waypoint]
        previous_waypoint = waypoint
    rows.append(current_row)
    
    if reverse_rows:
        rows = list(reversed(rows))

    path = [start]
    current_waypoint = start
    
    for row in rows:
        if current_waypoint not in row:
            row_left, row_right = min(row, key=lambda x: x[0]), max(row, key=lambda x: x[0])

            if current_waypoint[0] < row_left[0]:
                closest = row[0]
            elif current_waypoint[0] > row_right[0]:
                closest = row[-1]
            else:
                closest = min([(euclidean_dist(current_waypoint, wp), wp) for wp in (row[0], row[-1])], key=lambda x: x[0])[1]
            path.append(closest)
            current_waypoint = closest
        
        closest = max([(euclidean_dist(current_waypoint, wp), wp) for wp in row], key=lambda x: x[0])[1]

        i = row.index(current_waypoint)
        if i < row.index(closest):
            path += row[i:row.index(closest)+1]
        else:
            path += row[row.index(closest):i+1][::-1]

        path.append(closest)
        current_waypoint = closest
    
    path.append(start)

    return path

def get_rect_dim(poly_coord):
    min_x = min(coord[0] for coord in poly_coord)
    max_x = max(coord[0] for coord in poly_coord)
    min_y = min(coord[1] for coord in poly_coord)
    max_y = max(coord[1] for coord in poly_coord)

    w = max_x - min_x
    h = max_y - min_y

    if w > h:
        size_x = size_y = w
    else:
        size_x = size_y = h

    return (size_x, size_y)

def find_distance(points):
    x, y = points[0]

    prev_x, prev_y = x, y

    total_distance = 0
    for x, y in points:
        distance = ((x - prev_x) ** 2 + (y - prev_y) ** 2) ** 0.5
        total_distance += distance

        prev_x, prev_y = x, y

    return total_distance

def find_starting_wp(waypoints, starting_point):
    closest_dist = math.inf
    closest_wp = None

    for wp in waypoints:
        dist = euclidean_dist(wp, starting_point)
        if dist < closest_dist:
            closest_dist = dist
            closest_wp = wp
        
    return closest_wp

def convert_geographical_to_cartesian_2d(latitude, longitude):
    lat_rad = math.radians(latitude)
    long_rad = math.radians(longitude)

    R = 6371.0
    x = R * long_rad * math.cos(lat_rad)
    y = R * lat_rad

    return (y, x)

def convert_cartesian_to_geographical_2d(x, y):
    R = 6371.0
    lat_rad = math.asin(y / R)
    longitude = math.degrees(x / (R * math.cos(lat_rad)))
    latitude = math.degrees(lat_rad)

    return (longitude, latitude)

def find_corner_waypoints(waypoints):
    hull = ConvexHull(waypoints)
    
    return [tuple(waypoints[i]) for i in hull.vertices]

if __name__ == '__main__':
    drone_height = float(input("Wysokosc [m]: "))
    focal_length = float(input("Dlugosc ogniskowej [mm]: "))
    sensor_length = float(input("Szerokosc matrycy [mm]: "))
    sensor_height = float(input("Wysokosc matrycy [mm]: "))
    #coverage = input("Nakladanie sie zdjec [%]: ")

    starting_point = []

    starting_point_x = float(input("Punkt startowy szerokosc [stopnie]: "))
    starting_point_y = float(input("Punkt startowy dlugosc [stopnie]: "))

    starting_point = convert_geographical_to_cartesian_2d(starting_point_x, starting_point_y)
    print("Start wejsciowy: " + str(starting_point))

    #FOV = (sensor size * working distance) / focal length
    fov = []

    fov_x = (sensor_length * drone_height) / focal_length
    fov_y = (sensor_height * drone_height) / focal_length

    fov.append(fov_x)
    fov.append(fov_y)

    print("FOV [m]: " + str(fov))

    fov[0] = fov[0] / 1000
    fov[1] = fov[1] / 1000
    
    poly_coord = []
    poly_coord_str = input("Koordynaty wierzcholkow (szerokosc, dlugosc): ")

    for i in range(0, len(poly_coord_str.split()), 2):
        x, y = float(poly_coord_str.split()[i]), float(poly_coord_str.split()[i + 1])
        x, y = convert_geographical_to_cartesian_2d(x, y)
        poly_coord.append((x, y))

    print("Koordynaty wierzcholkow (x, y) [km]: " + str(poly_coord))

    angles = get_angles(poly_coord)

    print("Katy: " + str(angles) + "\n")

    sc = turtle.Screen()
    sc.setup(SCREEN_WIDTH, SCREEN_HEIGHT)

    t = turtle.Turtle()
    t.speed('fastest')

    t.width(2)
    t.hideturtle()

    n_waypoints = 0
    waypoints_poly = []
    
    sum_angles = (len(poly_coord) - 2) * 180
    x_axis_angle = 180 - get_angle(([-SCREEN_WIDTH / 2, 0], [SCREEN_WIDTH / 2, 0]), [poly_coord[0], poly_coord[1]])

    rotation_angle = x_axis_angle

    rect_coord = get_rect(poly_coord)
    rect_dim = get_rect_dim(rect_coord)

    sum_wp = 0

    should_draw = False
    paths = []
    rotations = []
    cycles = []
    nets = []

    starting_wp = None

    for i in range(len(poly_coord)):
        t0 = time.time()

        rotations.append(rotation_angle)
        net_waypoints = get_waypoints(poly_coord, rotate_polygon(rect_coord, rotation_angle), fov, "green", should_draw, rect_dim, starting_wp, rotation_angle)

        rotation_angle += 180 - angles[i]

        n_wp = len(net_waypoints)
        sum_wp += n_wp

        starting_wp = find_starting_wp(find_corner_waypoints(net_waypoints), starting_point)

        cycle = find_path(net_waypoints, starting_wp)
        
        distance = find_distance(cycle)

        print("Liczba waypointow, dlugosc sciezki i kat rotacji dla siatki nr " + str(i + 1) + ": " + str(n_wp) + " | " + str(distance) + " | " + str(rotations[i]))

        paths.append(distance)
        cycles.append(cycle)

        net_waypoints.clear()
        
        t1 = time.time()
        total = t1 - t0

        print("Czas: " + str(total))
    
    path_index = paths.index(min(paths))

    starting_wp = find_starting_wp(find_corner_waypoints(cycles[path_index]), starting_point)

    print("\nNajkrotsza jest sciezka nr " + str(path_index + 1) + " o dlugosci " + str(paths[path_index]))
    print("Sciezka zaczyna sie na: " + str(starting_wp))
    
    should_draw = True

    draw_polygon(poly_coord, "black")
    get_waypoints(poly_coord, rotate_polygon(rect_coord, rotations[path_index]), fov, "green", should_draw, rect_dim, starting_wp, rotations[path_index])

    path = list(dict.fromkeys(cycles[path_index]))
    #print("Corner " + str(find_corner_waypoints(path)))

    path.append(starting_wp)

    draw_path(path)

    for i in range(len(path)):
        path[i] = convert_cartesian_to_geographical_2d(path[i][0], path[i][1])

    print("\nWaypointy (szerokosc, dlugosc): " + str(path))
    print("Suma wszystkich waypointow: " + str(len(path)))
        
    turtle.update()
    turtle.done()
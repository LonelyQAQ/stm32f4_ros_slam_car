import math


def project_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 map_laser):
    tx, ty, yaw = map_laser
    c, sn = math.cos(yaw), math.sin(yaw)
    points = []
    for i, distance in enumerate(ranges):
        if math.isfinite(distance) and range_min <= distance <= range_max:
            angle = angle_min + i * angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append(tx + c * x - sn * y)
            points.append(ty + sn * x + c * y)
    return points

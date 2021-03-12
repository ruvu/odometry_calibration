import numpy as np
from angles import normalize_angle
from numpy import sin, cos
from tf.transformations import translation_matrix, quaternion_matrix, translation_from_matrix, \
    euler_from_matrix, euler_matrix


def to_matrix(data):
    """Convert a geometry_msgs.msg.TransformStamped to a 4d numpy transform matrix"""
    t = data.transform.translation
    r = data.transform.rotation
    t = translation_matrix((t.x, t.y, t.z))
    r = quaternion_matrix((r.x, r.y, r.z, r.w))
    return np.dot(t, r)


def pairs(seq):
    """Iterate over pairs in a list"""
    i = iter(seq)
    prev = next(i)
    for item in i:
        yield prev, item
        prev = item


def inverse_odometry(transform1, transform2):
    """
    Given two transforms, reverse calculate the odometry of the wheels for a differential drive robot

    Returns:
        A tuple of translation and rotation
    """
    delta_measurement = translation_from_matrix(transform2) - translation_from_matrix(transform1)
    r1 = euler_from_matrix(transform1)[2]
    r2 = euler_from_matrix(transform2)[2]

    R = np.linalg.lstsq([[sin(r2) - sin(r1)], [-cos(r2) + cos(r1)]], delta_measurement[:2], rcond=None)[0][0]
    angular = normalize_angle(r2 - r1)
    linear = R * angular

    return linear, angular


def _odometry_runge_kutta2(position, linear, angular):
    x, y = translation_from_matrix(position)[:2]
    heading = euler_from_matrix(position)[2]
    direction = heading + angular * 0.5

    # Runge-Kutta 2nd order integration:
    x += linear * cos(direction)
    y += linear * sin(direction)
    heading += angular
    return x, y, heading


def _odometry_exact(position, linear, angular):
    # Exact integration (should solve problems when angular is zero):
    x, y = translation_from_matrix(position)[:2]
    heading = euler_from_matrix(position)[2]

    heading_old = heading
    r = linear / angular
    heading += angular
    x += r * (sin(heading) - sin(heading_old))
    y += -r * (cos(heading) - cos(heading_old))
    return x, y, heading


def odometry(position, linear, angular):
    """
    Reimplementation of the odometry calculation from diff_drive_controller

    Given a starting position, forward and angular displacement, calculate the end position.
    """
    if abs(angular) < 1e-6:
        x, y, heading = _odometry_runge_kutta2(position, linear, angular)
    else:
        x, y, heading = _odometry_exact(position, linear, angular)

    t = translation_matrix((x, y, 0))
    r = euler_matrix(0, 0, heading)
    return np.dot(t, r)

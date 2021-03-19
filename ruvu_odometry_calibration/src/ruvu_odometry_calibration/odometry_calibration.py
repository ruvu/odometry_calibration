from dataclasses import dataclass
from typing import Any

import numpy as np
from angles import normalize_angle
from angles import shortest_angular_distance
from numpy import sin, cos
from ruvu_odometry_calibration.tf2_rosbag import BagBuffer
from tf.transformations import inverse_matrix, identity_matrix
from tf.transformations import translation_matrix, quaternion_matrix, translation_from_matrix, \
    euler_from_matrix, euler_matrix
from tf2_ros import ExtrapolationException


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


@dataclass
class DataPoint:
    timestamp: Any
    ground_truth: Any
    measurement: Any


def bag_to_datapoints(bag):
    """
    Load all data of a bagfile into a list of DataPoints
    """
    bag_transformer = BagBuffer(bag)

    data = []
    for _, msg, _ in bag.read_messages(topics=['/scan']):
        try:
            ground_truth = bag_transformer.lookup_transform('map', 'base_link', msg.header.stamp)
            measurement = bag_transformer.lookup_transform('odom', 'base_link', msg.header.stamp)
        except ExtrapolationException:
            continue
        ground_truth = to_matrix(ground_truth)
        measurement = to_matrix(measurement)
        data.append(DataPoint(timestamp=msg.header.stamp, ground_truth=ground_truth, measurement=measurement))
    return data


@dataclass
class Parameters:
    wheel_separation_multiplier: float
    wheel_radius_multiplier: float


def loss(data, parameters):
    """
    Loss function

    Given a bunch of data, and a set of parameters, return residual errors. Row in the residuals is a tuple of errors
    (x, y, theta)
    """
    errs = []
    for d1, d2 in pairs(data):
        linear, angular = inverse_odometry(d1.measurement, d2.measurement)

        linear2 = linear * parameters.wheel_radius_multiplier
        angular2 = angular * parameters.wheel_radius_multiplier / parameters.wheel_separation_multiplier

        new_pos = odometry(identity_matrix(), linear2, angular2)

        # x and y error
        delta_ground_truth = np.dot(inverse_matrix(d1.ground_truth), d2.ground_truth)
        err_2d = translation_from_matrix(delta_ground_truth)[:2] - translation_from_matrix(new_pos)[:2]

        # rotation error
        err_rot = shortest_angular_distance(euler_from_matrix(delta_ground_truth)[2], euler_from_matrix(new_pos)[2])
        errs.append((err_2d[0], err_2d[1], err_rot))

    errs = np.array(errs)
    return errs

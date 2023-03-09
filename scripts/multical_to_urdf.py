#!/usr/bin/env python3
"""\
This script converts the output of MultiCal (a folder containing files for 
the extrinsics of the IMU, cameras and the LiDAR) to an URDF-readable format.
Specifically, it stores all extrinsic transforms in a single yaml file,
additionally computing the fixed-angle Euler Axis Representation as used
by the URDF format.
"""
import numpy as np
import tf.transformations
import os
import yaml
import argparse
import rospkg


def log_error(text):
    print(f"\033[91mERROR\033[0m: {text}")


if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description='Convert the output of MultiCal to a URDF calibration.')
    parser.add_argument('multical_results_path',
                        help='The path to MultiCals output folder.')
    parser.add_argument('--output_name', default="calibration.yaml",
                        help='The name of the output file.')
    args = parser.parse_args()
    calib_path = args.multical_results_path
    output_name = args.output_name

    # Obtain calibration result files
    if not os.path.isdir(calib_path):
        log_error('Provided multical_results_path is not a directory!')
        exit()
    imus_path, lidars_path, cams_path = None, None, None
    for filename in os.listdir(calib_path):
        if filename.endswith("imus.yaml"):
            if imus_path is not None:
                log_error('Multiple IMU calibrations present in folder!')
                exit()
            imus_path = os.path.join(calib_path, filename)
        if filename.endswith('lidars.yaml'):
            if lidars_path is not None:
                log_error('Multiple LiDAR calibrations present in folder!')
                exit()
            lidars_path = os.path.join(calib_path, filename)
        if filename.endswith('cams.yaml'):
            if cams_path is not None:
                log_error('Multiple Camera calibrations present in folder!')
                exit()
            cams_path = os.path.join(calib_path, filename)
    if imus_path is None:
        log_error('Unable to find IMU calibration!')
    if lidars_path is None:
        log_error('Unable to find LiDAR calibration!')
    if cams_path is None:
        log_error('Unable to find Camera calibration!')
    if None in [imus_path, lidars_path, cams_path]:
        exit()

    # Perform conversion
    output = {}
    for filepath in [imus_path, lidars_path, cams_path]:
        with open(filepath, 'r') as stream:
            data = yaml.safe_load(stream)
        keys = data.keys()
        for key in keys:
            if key == 'imu0':
                continue
            print(f'===== T_imu0_{key} =====')
            T_sensor_imu0 = np.array(data[key]['T_here_imu0'])
            T_imu0_sensor = np.linalg.inv(T_sensor_imu0)
            roll, pitch, yaw = tf.transformations.euler_from_matrix(
                T_imu0_sensor[0:3, 0:3], 'sxyz')
            x, y, z = T_imu0_sensor[0:3, 3]

            qx, qy, qz, qw = tf.transformations.quaternion_from_matrix(
                T_imu0_sensor)

            print(f'xyz=\"{x} {y} {z}\"')
            print(f'rpy=\"{roll} {pitch} {yaw}\"')
            print(f'q(xyzw)=\"{qx} {qy} {qz} {qw}\"')
            print()

            output_i = {'x': x.item(), 'y': y.item(), 'z': z.item(
            ), 'roll': roll, 'pitch': pitch, 'yaw': yaw}
            output[f'T_imu0_{key}'] = output_i

    # Store result
    rp = rospkg.RosPack()
    package_path = rp.get_path('trailblazer_description')
    output_path = os.path.join(package_path, 'config', output_name)
    if not output_path.endswith('.yaml'):
        output_path += '.yaml'
    with open(output_path, 'w') as file:
        documents = yaml.dump(output, file)

    print(f'Saved results to {output_path}')

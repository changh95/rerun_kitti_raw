import rerun as rr
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import pykitti

__author__ = "Hyunggi Chang (changh95)"


def main():
    # parse arguments
    parser = argparse.ArgumentParser(description="Rerun visualization of KITTI raw dataset")
    parser.add_argument("--basedir", type=str, help="Path to the KITTI raw dataset")
    parser.add_argument("--date", type=str, help="Date of the sequence")
    parser.add_argument("--drive", type=str, help="Drive of the sequence")
    args = parser.parse_args()

    # Load the dataset
    dataset = pykitti.raw(args.basedir, args.date, args.drive)

    # Initialize rerun
    rr.init("KITTI_raw", spawn=True)
    rr.spawn()
    rr.set_time_seconds("stable_time", 0)

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, timeless=True)

    # Log LiDAR extrinsic calibration
    T_velo_imu = np.linalg.inv(dataset.calib.T_velo_imu.reshape(4, 4))
    R_velo_imu = R.from_matrix(T_velo_imu[:3, :3]).as_quat()
    rr.log("world/ego_vehicle/lidar",
           rr.Transform3D(translation=T_velo_imu[:3, 3], rotation=rr.Quaternion(xyzw=R_velo_imu)))

    # Log Camera extrinsic calibration
    T_cam0_imu = np.linalg.inv(dataset.calib.T_cam0_imu.reshape(4, 4))
    R_cam0_imu = R.from_matrix(T_cam0_imu[:3, :3]).as_quat()
    rr.log("world/ego_vehicle/cam0",
           rr.Transform3D(translation=T_cam0_imu[:3, 3], rotation=rr.Quaternion(xyzw=R_cam0_imu)))

    T_cam1_imu = np.linalg.inv(dataset.calib.T_cam1_imu.reshape(4, 4))
    R_cam1_imu = R.from_matrix(T_cam1_imu[:3, :3]).as_quat()
    rr.log("world/ego_vehicle/cam1",
           rr.Transform3D(translation=T_cam1_imu[:3, 3], rotation=rr.Quaternion(xyzw=R_cam1_imu)))

    T_cam2_imu = np.linalg.inv(dataset.calib.T_cam2_imu.reshape(4, 4))
    R_cam2_imu = R.from_matrix(T_cam2_imu[:3, :3]).as_quat()
    rr.log("world/ego_vehicle/cam2",
           rr.Transform3D(translation=T_cam2_imu[:3, 3], rotation=rr.Quaternion(xyzw=R_cam2_imu)))

    T_cam3_imu = np.linalg.inv(dataset.calib.T_cam3_imu.reshape(4, 4))
    R_cam3_imu = R.from_matrix(T_cam3_imu[:3, :3]).as_quat()
    rr.log("world/ego_vehicle/cam3",
           rr.Transform3D(translation=T_cam3_imu[:3, 3], rotation=rr.Quaternion(xyzw=R_cam3_imu)))

    # Log camera intrinsic calibration
    width = 2 * 6.018873e+02
    height = 2 * 1.831104e+02
    rr.log("world/ego_vehicle/cam0", rr.Pinhole(focal_length=float(dataset.calib.K_cam0[0, 0]), width=width, height=height))
    rr.log("world/ego_vehicle/cam1", rr.Pinhole(focal_length=float(dataset.calib.K_cam1[0, 0]), width=width, height=height))
    rr.log("world/ego_vehicle/cam2", rr.Pinhole(focal_length=float(dataset.calib.K_cam2[0, 0]), width=width, height=height))
    rr.log("world/ego_vehicle/cam3", rr.Pinhole(focal_length=float(dataset.calib.K_cam3[0, 0]), width=width, height=height))

    for i in range(len(dataset.timestamps)):
        # Set timestamp
        timestamp = dataset.timestamps[i]
        rr.set_time_seconds("stable_time", timestamp.timestamp())

        # Log oxts (ego-vehicle pose)
        translation = dataset.oxts[i].T_w_imu[:3, 3]
        rotation = dataset.oxts[i].T_w_imu[:3, :3]
        rotation_xyzw = R.from_matrix(rotation).as_quat()
        rr.log("world/ego_vehicle", rr.Transform3D(translation=translation, rotation=rr.Quaternion(xyzw=rotation_xyzw)))

        # Log LiDAR
        lidar = dataset.get_velo(i)
        rr.log("world/ego_vehicle/lidar", rr.Points3D(lidar[:, :3], colors=[1.0, 1.0, 1.0]))

        # Log gray cameras
        cam0, cam1 = dataset.get_gray(i)
        rr.log("world/ego_vehicle/cam0", rr.Image(np.array(cam0)).compress(jpeg_quality=75))
        rr.log("world/ego_vehicle/cam1", rr.Image(np.array(cam1)).compress(jpeg_quality=75))

        # Log RGB cameras
        cam2, cam3 = dataset.get_rgb(i)
        rr.log("world/ego_vehicle/cam2", rr.Image(np.array(cam2)).compress(jpeg_quality=75))
        rr.log("world/ego_vehicle/cam3", rr.Image(np.array(cam3)).compress(jpeg_quality=75))


if __name__ == "__main__":
    main()

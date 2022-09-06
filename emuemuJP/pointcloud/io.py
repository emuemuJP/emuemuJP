import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2

def read_pcds_from_bag(rosbag, pcd_topic:str, device=o3d.core.Device("CPU:0")):
    pcds = []
    for topic, msg, t in rosbag.read_messages():
        if topic == pcd_topic:
            pcd = np.array(list(pc2.read_points(msg)))
            _pcd = o3d.t.geometry.PointCloud(device)
            _pcd.point["positions"] = o3d.core.Tensor(pcd[:, :3], device=device)
            _pcd.point["intensities"] = o3d.core.Tensor(pcd[:, 3], device=device)
            pcds.append(_pcd)
    return pcds
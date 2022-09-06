import numpy as np

def min_max_range(pcd_numpy):
    pts_endpoints = np.array([[pcd_numpy[:, i].min(), pcd_numpy[:, i].max()] for i in range(pcd_numpy.shape[1])])
    return pts_endpoints, pts_endpoints.min(), pts_endpoints.max()

def get_thickness_for_given_direction(pts, direction):
    ds = -np.dot(pts, direction)
    return ds.max() - ds.min(), np.array([pts[ds.argmin()], pts[ds.argmax()]]), np.array([ds.min(), ds.max()])

def calc_distance_pcds_center(src_pcd, dist_pcd):
    return np.linalg.norm(src_pcd.get_center() - dist_pcd.get_center())
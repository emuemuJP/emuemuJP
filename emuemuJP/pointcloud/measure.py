import numpy as np

def min_max_range(pcd_numpy):
    pts_endpoints = np.array([[pcd_numpy[:, i].min(), pcd_numpy[:, i].max()] for i in range(pcd_numpy.shape[1])])
    return pts_endpoints, pts_endpoints.min(), pts_endpoints.max()

# return thickness, endpoints, endpoints_values
def get_thickness_for_given_direction(pts, direction):
    ds = -np.dot(pts, direction)
    thickness = ds.max() - ds.min()
    endpoints = np.array([pts[ds.argmin()], pts[ds.argmax()]])
    endpoints_values = np.array([ds.min(), ds.max()])
    return thickness, endpoints, endpoints_values

def calc_pcds_distance_center(src_pcd, dist_pcd):
    return np.linalg.norm(src_pcd.get_center() - dist_pcd.get_center())

def calc_pcds_distance_to_direction_of_normal(src_pcd, dist_pcd):
    src_pts = np.array(src_pcd.points)
    src_pts_normal = np.array(src_pcd.normals)
    dist_pts = np.array(dist_pcd.points)
    dists = np.array([])
    for i in range(src_pts.shape[0]):
        # A + dot(AP,AB) / dot(AB,AB) * AB
        line_pts = src_pts[i] + np.dot((dist_pts - src_pts[i]), src_pts_normal[i][np.newaxis].T)
        dists2line = np.linalg.norm(line_pts - dist_pts, axis=1)
        inliers = np.where(np.abs(dists2line) <= 0.2)[0]
        signed_dist = (line_pts[inliers] - src_pts[i])[:, 0]
        # positive_inliers = np.where(signed_dist > 0)[0]
        if len(inliers)==0: dists = np.append(dists, 1)
        else: dists = np.append(dists, signed_dist.max())
    return dists

def calc_dist_pt2pts(origin, pts):
    return np.linalg.norm((pts[:] - origin), axis=1)

def calc_dist_pts2plane(pts, plane_eq):
    return (plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] + plane_eq[2] * pts[:, 2] + plane_eq[3]) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

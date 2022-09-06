import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import random
import copy
from . import measure

def clustering(pcd, show=False):
    labels = np.array(pcd.cluster_dbscan(0.75, 5, print_progress=True))
    print(f"point cloud has {labels.max() + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (labels.max() if labels.max() > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    if show: o3d.visualization.draw_geometries([pcd], "Open3D dbscanclusting")
    return pcd, labels

def select_cluster(pcds, labels, cluster):
    return [pcds[index] for index, label in enumerate(labels) if label==cluster]

def sampling_and_cluster_filtering(pcd, vsize, with_normals=True):
    if vsize == 0: voxel_down_pcd = pcd
    else: voxel_down_pcd = pcd.voxel_down_sample(voxel_size=vsize)
    voxel_down_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=vsize*15, max_nn=100))
    clustered_pcd, labels = clustering(voxel_down_pcd, False)
    clustered_pcd_as_array = np.asarray(clustered_pcd.points)
    selected_pcd = o3d.geometry.PointCloud()
    selected_pcd.points = o3d.utility.Vector3dVector(select_cluster(clustered_pcd_as_array, labels, np.array([len(np.where(labels==label_idx)[0]) for label_idx in range(labels.max()+1)]).argmax()))
    if with_normals: selected_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=vsize*15, max_nn=100))
    selected_pcd.paint_uniform_color([0, 0, 0])
    return selected_pcd

def ransac_normal(pcd, sample_points=10, iteration=100):
    min_error = np.Inf
    min_normal = None
    pcd_normals = np.asarray(pcd.normals)
    n_points = pcd_normals.shape[0]
    for _ in range(iteration):
        id_samples = random.sample(range(0, n_points), 2)
        id_samples_tunnel = random.sample(range(0, n_points), sample_points)
        normal_samples = pcd_normals[id_samples]
        normal = np.cross(normal_samples[0], normal_samples[1])
        normal_norm = normal/np.linalg.norm(normal)
        normal_stack = np.stack([normal_norm] * sample_points, 0)
        tunnel_sample_normals = pcd_normals[id_samples_tunnel]
        error = np.sum(abs(np.dot(normal_stack, tunnel_sample_normals.T)))
        if min_error > error :
            min_error = error
            min_normal = normal_norm
    return min_normal


def convert_numpy2pcd(pcd_numpy, vsize=0.2):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_numpy)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=vsize*15, max_nn=100))
    pcd.paint_uniform_color([0, 0, 0])
    return pcd

def remove_specific_vector_plane(pcd, vector, threshold=0.6):
    pts = np.array([pcd.points])[0]
    best_eq = []
    best_inliers = []
    pts_endpoints, _min, _max = measure.min_max_range(pts)
    maxIteration=1000
    for it in range(maxIteration):
        d = random.uniform(_min, _max)
        plane_eq = [vector[0], vector[1], vector[2], d]

        # Distance from a point to a plane
        # https://mathworld.wolfram.com/Point-PlaneDistance.html
        pt_id_inliers = []  # list of inliers ids
        dist_pt = (
            plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] + plane_eq[2] * pts[:, 2] + plane_eq[3]
        ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

        # Select indexes where distance is biggers than the threshold
        pt_id_inliers = np.where(np.abs(dist_pt) <= threshold)[0]
        if len(pt_id_inliers) > len(best_inliers):
            best_eq = plane_eq
            best_inliers = pt_id_inliers
    pt_inliers_plane = pts[best_inliers]
    ind = np.ones(pts.shape[0], dtype=bool)
    ind[best_inliers] = False
    pt_outliers = pts[ind]
    return convert_numpy2pcd(pt_outliers), convert_numpy2pcd(pt_inliers_plane), best_eq

def get_points_within_plane(pcd, plane_eq, threshold = 2.0):
    pts = np.array([copy.deepcopy(pcd).points])[0]
    dist_pt = (
        plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] + plane_eq[2] * pts[:, 2] + plane_eq[3]
    ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

    # Select indexes where distance is biggers than the threshold
    pt_inliers = np.where(np.abs(dist_pt) <= threshold)[0]
    pt_outliers = np.ones(pts.shape[0], dtype=bool)
    pt_outliers[pt_inliers] = False
    return convert_numpy2pcd(pts[pt_outliers]), convert_numpy2pcd(pts[pt_inliers])


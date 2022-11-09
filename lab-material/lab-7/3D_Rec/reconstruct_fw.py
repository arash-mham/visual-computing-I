import numpy as np
import open3d as o3d


from utils import find_distance, run_marching_cubes


def find_knn(points, k):
    # points: numpy array, point set with size (N, d)
    # k: int, number of nearest neighbors
    # return: numpy array, knn index with size (N, k)
    # todo: use the find_distance function to find the distance matrix, Then find the k nearest neighbors for each point. Return the knn indexes for each point (Except for the point itself)
    # useful functions: np.argsort
    raise NotImplementedError

def find_normals(points, k):
    # points: numpy array, point set with size (N, d)
    # k: int, number of nearest neighbors
    # return: numpy array, normal of point set with size (N, d)
    # useful functions: np.linalg.svd
    knn = find_knn(points, k)

    normal = np.zeros(points.shape)
    for i in range(points.shape[0]):
        neighbor = points[knn[i]]
        # todo: Using SVD find the normals of each point. Return the normal of each point.
    raise NotImplementedError
    return normal


def sdf(queries, pointcloud, centers):
    # queries: numpy array, query point set with size (N, d)
    # points: Open3D pointcloud
    # return: numpy array, sdf of query points

    normals = np.asarray(pointcloud.normals)

    # todo: find closest center

    # todo: get closest normal


    # todo: find sdf
    raise NotImplementedError
    return sdf


if __name__ == '__main__':
    ##Make sphere point cloud
    num_points = 2048
    r = 1
    # todo: Sample points on a sphere with radius 1
    points = None

    # Convert to open3D pointcloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Visualize point cloud
    # o3d.visualization.draw_geometries([pcd])

    # todo: Estimate normals. Complete the find_normals function
    normals = find_normals(points, 30)
    # Set Open3D point cloud normals
    pcd.normals = o3d.utility.Vector3dVector(normals)

    # Visualize point cloud with normals
    # o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    # Correct normal orientations
    pcd.orient_normals_consistent_tangent_plane(k=15)

    # Find tangent plane centers
    knn = find_knn(points, 15)
    centers = points[knn].mean(axis=1)

    # Run Marching cubes
    run_marching_cubes(sdf, pcd, centers, resolution=50)




import numpy as np
import trimesh
from skimage import measure

def get_grid_uniform(resolution, grid_boundry = [-2.0, 2.0]):
    x = np.linspace(grid_boundry[0], grid_boundry[1], resolution)
    y = x
    z = x

    xx, yy, zz = np.meshgrid(x, y, z)
    grid_points = np.stack([xx, yy, zz], axis=-1).reshape(-1, 3)
    return {'grid_points': grid_points,
            'xyz': [x, y, z]}

def run_marching_cubes(implicit_function, pointcloud, centers, resolution=256, grid_boundry=[-2., 2.]):
    grid = get_grid_uniform(resolution, grid_boundry)
    points = grid['grid_points']
    xyz = grid['xyz']
    sdfs = []
    for i, pset in enumerate(np.split(points, 1000)):
        sdf = implicit_function(pset, pointcloud, centers)
        sdfs.append(sdf)
        print(i)
    sdfs = np.concatenate(sdfs)
    #sdf_values = implicit_function(points, tpcs, normals)
    z = sdfs.astype(np.float32)

    verts, faces, normals, values = measure.marching_cubes_lewiner(z.reshape(resolution, resolution, resolution).transpose([1, 0, 2]), 0.0,
                                                           spacing=(xyz[0][2] - xyz[0][1],
                                                                    xyz[0][2] - xyz[0][1],
                                                                    xyz[0][2] - xyz[0][1]))
    verts = verts + np.array([xyz[0][0], xyz[1][0], xyz[2][0]])
    meshexport = trimesh.Trimesh(verts, faces, normals)
    meshexport.export('surface.ply', 'ply')

def find_distance(x, y):
    #x: numpy array, point set with size (N, d)
    #y: numpy array, point set with size (M, d)
    #return: distance matrix with size (N, M)
    xx = np.repeat(np.sum(x * x, axis=1), y.shape[0]).reshape(x.shape[0], y.shape[0])
    yy = np.repeat(np.sum(y * y, axis = 1), x.shape[0]).reshape(y.shape[0], x.shape[0]).T
    xy = np.matmul(x, y.T)
    dist = np.sqrt(np.clip(xx + yy - 2 * xy, 0, None))
    return dist
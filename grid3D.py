import numpy as np
import matplotlib.pyplot as plt

def get_asymmetric_grid_3D(grid_size=(4,11), flip_start_end=False, lidar_mode=False):

    points3D = []
    z = np.zeros((grid_size[0], 1))
    for i in range(grid_size[1]):
            x = i * np.ones((grid_size[0], 1))
            if i % 2 == 0:
                y = np.arange(1, 2 * grid_size[0], 2, dtype=float).reshape(-1, 1)
            else:
                if i != grid_size[1]-1:
                    y = np.arange(0, 2 * grid_size[0] - 1, 2, dtype=float).reshape(-1, 1)
                
            row = np.hstack([x, y, z])
            if i == 0:
                points3D = row
            else:
                points3D = np.vstack([points3D, row])
                
    if flip_start_end:
        points3D = np.flipud(points3D)
        
    if lidar_mode:
        new_points = np.zeros(points3D.shape)
        new_points[:,0] = points3D[:,2]
        new_points[:,1] = points3D[:,0]
        new_points[:,2] = points3D[:,1]
        points3D = new_points

    return points3D.astype(np.float32)

def plot_grid(points3D):

    plt.figure()
    plt.plot(points3D[:,0], points3D[:,1], '-o')
    plt.show()

                
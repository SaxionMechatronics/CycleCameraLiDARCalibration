import numpy as np
import random

import random
import matplotlib.pyplot as plt

def prefilter_pointcloud(cloud, max_heading, max_depth):

    headings = np.arctan2(cloud[:,1], cloud[:,0]) / np.pi * 180.0
    cloud = cloud[np.abs(headings) < max_heading, :]
    cloud = cloud[cloud[:,0] < max_depth]

    return cloud

def find_3_closest_planes(planes):

    centers = []
    for plane in planes:
        center = np.mean(plane, axis=0)
        centers.append(center)

    centers = np.array(centers)
    
    # print(centers.shape)
    dists_to_origin = np.linalg.norm(centers, axis=1)
    # print(dists_to_origin.shape)
    closest_planes = np.argsort(dists_to_origin)[:3]

    return closest_planes

def distance_to_plane(point, plane):
    denom = np.sqrt(plane[0]**2 + plane[1]**2 + plane[2]**2)
    return np.abs(plane[0]*point[0] + plane[1]*point[1] + plane[2]*point[2] + plane[3]) / denom

def refine_plane_boundaries(points, plane_eqs):

    if len(plane_eqs) < 2:
        print("Requires 2 or 3 plane equations")
        return
    if len(plane_eqs) > 3:
        print("Too many plane equations. Requires 2 or 3")
        return
    
    plane1 = []
    plane2 = []
    plane3 = []
    new_planes = [plane1, plane2, plane3]
    new_colors = []

    if len(plane_eqs) == 2:
        new_planes = new_planes[:2]

    for point in points:
        best_plane = 0
        smallest_distance = 9999.9
        for i, w in enumerate(plane_eqs):
            dist = distance_to_plane(point, w)
            # print(dist)
            if dist < smallest_distance:
                smallest_distance = dist
                best_plane = i
        new_planes[best_plane].append(point)
        # print("-------------------")

    for i in range(len(new_planes)):
        new_planes[i] = np.array(new_planes[i])

        r = random.random()
        g = random.random()
        b = random.random()

        color = np.zeros((new_planes[i].shape[0], 3))
        color[:, 0] = r # 0.5
        color[:, 1] = g # 0.5
        color[:, 2] = b # 0.5
        new_colors.append(color)

    # print(new_planes[0].shape)
    # print(new_planes[1].shape)

    return new_planes, new_colors

def removeNoisePlanes(planes, centers, threshold):

    filtered_planes = []
    for i, plane in enumerate(planes):
        dists_to_center = np.linalg.norm(plane - centers[i], axis=1)
        filtered_plane = plane[dists_to_center < threshold]
        filtered_planes.append(filtered_plane)

    return filtered_planes


def find_plane_order(plane_eqs, centers):
    
    n1 = plane_eqs[0][:3]
    n2 = plane_eqs[1][:3]
    n3 = plane_eqs[2][:3]
    
    p1 = centers[0]
    p2 = centers[1]
    p3 = centers[2]
    
    #  Check if all normal vectors are pointing inwards
    #  If dot product positive, both vectors are pointing in same direction.
    #  If negative, there is at least a 180 degree direction difference
    if np.dot(n1, (p2-p1)) < 0:
        n1 = -n1
    if np.dot(n2, (p3-p2)) < 0:
        n2 = -n2
    if np.dot(n3, (p1 - p3)) < 0:
        n3 = -n3
        
    normals = np.hstack([n1[:, np.newaxis], n2[:, np.newaxis], n3[:, np.newaxis]])
    centers_np = np.vstack(centers)
    
    # Compute absolute values of normal components
    abs_normals = np.abs(normals)
    
    # Identify the bottom plane (closest to the Z-axis)
    bottom_index = np.argmax(abs_normals[2])  # Plane with highest |n_z|
    
    # Remaining two planes (left and right)
    remaining_indices = [i for i in range(3) if i != bottom_index]
    # remaining_normals = normals[remaining_indices]
    remaining_centers = centers_np[remaining_indices]
    # print(remaining_centers)
    
    # Identify left and right based on y-component
    left_index = remaining_indices[np.argmax(remaining_centers[:,1])]  # More positive y
    right_index = remaining_indices[np.argmin(remaining_centers[:,1])]  # More negative y
    
    plane_order = [left_index, right_index, bottom_index]
    
    return plane_order

def find_2_planes(plane_eqs, centers):

    x_axis = np.array([1, 0, 0])
    
    n1 = plane_eqs[0][:3]
    n2 = plane_eqs[1][:3]
    
    p1 = centers[0]
    p2 = centers[1]

    # Create vectors from plane to origin
    ref_point = np.array([0, 0, 0])
    v1 = ref_point - p1
    v2 = ref_point - p2

    # Check if all the normals are pointing towards origin (and thus each other)
    if np.dot(n1, v1) < 0:
        n1 = -n1
    if np.dot(n2, v2) < 0:
        n2 = -n2

    # Compute angles between the normal vectors and the x-axis
    angle_n1_x = np.arccos(np.dot(n1, x_axis) / (np.linalg.norm(n1) * np.linalg.norm(x_axis)))
    angle_n2_x = np.arccos(np.dot(n2, x_axis) / (np.linalg.norm(n2) * np.linalg.norm(x_axis)))

    # Return target plane first, second plane after
    if angle_n1_x > angle_n2_x:
        return [0, 1]
    else:
        return [1, 0]
    
def reorder_planes(planes, plane_eqs, plane_order):
    
    new_planes = [planes[i] for i in plane_order]
    new_plane_eqs = [plane_eqs[i] for i in plane_order]
    
    new_colors = []
    
    for i in range(3):
        color = np.zeros((new_planes[i].shape[0], 3))
        color[:,i] = 1.0
        new_colors.append(color)
        
    return new_planes, new_colors, new_plane_eqs

def plane_intersection(n1, d1, n2, d2):
    # Compute the direction of the intersection line
    line_direction = np.cross(n1, n2)
    
    if np.linalg.norm(line_direction) == 0:
        raise ValueError("Planes are parallel and do not intersect.")
    
    # Solve linear least-square Ax=d for a point x on the intersection
    A = np.array([n1, n2, line_direction])  # Construct coefficient matrix
    d = np.array([d1, d2, 0])  # Right-hand side values

    # Solve Ax = d using linear least squares
    point_on_line = np.linalg.lstsq(A, d, rcond=None)[0]

    # if validate_point(n1, d1, n2, d2, -point_on_line) < validate_point(n1, d1, n2, d2, point_on_line):
    if point_on_line[0] < 0:
        # x-coordinate must always be positive (in front of the LiDAR)
        point_on_line = -point_on_line
    
    return point_on_line, line_direction

def line_intersection(p1, d1, p2, d2):
    
    # Create the system of equations
    # p1 + t1 * d1 = p2 + t2 * d2
    # Rearranged:
    # t1 * d1 - t2 * d2 = p2 - p1
    A = np.array([d1, -d2]).T  # Coefficient matrix (3x3)
    # print(A.shape)
    b = np.array(p2) - np.array(p1)  # Right-hand side vector

    # Solve for t1 and t2
    t1, t2 = np.linalg.lstsq(A, b, rcond=None)[0]
    # Compute the intersection point using t1 in the equation of the first line
    intersection = p1 + t1 * d1
    return intersection

        
def analyze_plane(points):
    
    # Compute the centroid
    centroid = np.mean(points, axis=0)

    # Center the points
    centered_points = points - centroid

    # Perform SVD
    _, _, vh = np.linalg.svd(centered_points)

    # The normal vector is the last row of vh
    normal_vector = vh[-1]
    v1 = np.expand_dims(vh[0],1)
    v2 = np.expand_dims(vh[1],1)

    # Normalize the normal vector
    normal_vector /= np.linalg.norm(normal_vector)
    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)

    print("v1: ", v1)
    print("v2: ", v2)
    print("----------------------")
    
    # Compute angles
    theta = np.arccos(normal_vector[2])  # Inclination angle
    phi = np.arctan2(normal_vector[1], normal_vector[0])  # Azimuth angle
    # roll = get_plane_roll(v1, normal_vector)
    
    local_coords = np.array([[np.dot(p, v1), np.dot(p, v2)] for p in points])
    # local_coords = rotate_2d_plane(local_coords, roll)
    
    # Step 3: Compute dimensions using bounding box
    u_min, u_max = local_coords[:, 0].min(), local_coords[:, 0].max()
    v_min, v_max = local_coords[:, 1].min(), local_coords[:, 1].max()
    
    width = u_max - u_min
    height = v_max - v_min

    # print(theta / np.pi * 180.0)
    # print(phi / np.pi * 180.0)
    # print(roll / np.pi * 180.0)
    print(width)
    print(height)
    print("----------------")
    
    plane_props = {
        "centroid": centroid,
        "width": width,
        "height": height,
        "theta": theta,
        "phi": phi
    }
    
    return plane_props
    
def reorder_list(list, order):

    new_list = []
    for idx in order:
        new_list.append(list[idx])

    return new_list

def shift_to_left(list):

    N = len(list)
    new_list = [0] * N
    for i, item in enumerate(list):
        new_idx = (i - 1) % N
        new_list[new_idx] = item
    return new_list


def set_axes_equal(ax, set_limits=False, axis_limits=[[0, 0], [0, 0], [0, 0]]):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    if set_limits:
        x_limits = axis_limits[0]
        y_limits = axis_limits[1]
        z_limits = axis_limits[2]
    else:
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def viz_line_points(point_on_line, line_dir, offset=0.5, dir_only=0):
    
    points = []
    if dir_only == 1:
        points = [point_on_line, point_on_line + offset * line_dir]
    elif dir_only == -1:
        points = [point_on_line, point_on_line - offset * line_dir]
    else:
        points = [point_on_line - offset * line_dir, point_on_line + offset * line_dir]
    return np.array(points)
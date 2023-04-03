import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

def uvtoXYZ(img, E_oc_inv):
    XYZ_all = []
    print(img.shape[0], img.shape[1])
    for h in range(img.shape[0]): 
            for w in range(img.shape[1]):
                # print(img[h,w])
                if img[h,w] == 255:
                    # wire_pixel_l.append((i,j))
                    uv11 = np.array([
                        [w],
                        [h],
                        [1],
                        [1]
                    ])
                    XYZ1 = np.dot(E_oc_inv,uv11)
                    XYZ = XYZ1.T[0][0:3]
                    XYZ_all.append(XYZ)
    return XYZ_all

def findintersection(set_l, set_r, tvec_l, tvec_r):
    tvec_l = tvec_l.T[0] / np.linalg.norm(tvec_l.T[0])
    tvec_r = tvec_r.T[0] / np.linalg.norm(tvec_r.T[0])

    cross_check = np.cross(tvec_l, tvec_r)
    intersect_points = []
    # intersect_points_ = []
    if (np.linalg.norm(cross_check) != 0):
        for XYZ_l in set_l:
            for XYZ_r in set_r:
                # Find the point of intersection
                coeff_matrix = np.vstack((tvec_l, -tvec_r)).T
                const_matrix = XYZ_r - XYZ_l
                t1, t2 = np.linalg.lstsq(coeff_matrix, const_matrix, rcond=None)[0]
                # intersect = XYZ_l + tvec_l * t
                intersect = XYZ_l + tvec_l * t1

                # intersect_ = XYZ_r + tvec_r * t2
                # print(t1, tvec_l, '1')
                # print(t2, tvec_r, '2')
                # print(intersect, intersect_, intersect-intersect_)
                # intersect = t
                intersect_points.append(intersect)
                # intersect_points_.append(intersect_)

                print(len(intersect_points), len(set_l)*len(set_r))
    else:
        print("The two vectors are parallel and do not intersect.")
        return 0
    
    return np.array(intersect_points)

def pointcloud2voxel(intersect_points, resolution):
    # Determine the bounding box of the point cloud
    min_coords = np.min(intersect_points, axis=0)
    max_coords = np.max(intersect_points, axis=0)
    bounding_box_size = max_coords - min_coords
    print(bounding_box_size)
    # Calculate the number of voxels needed in each dimension
    voxel_counts = np.ceil(bounding_box_size / resolution).astype(int)
    print(voxel_counts, 'voxel counts')
    # Create an empty voxel grid
    voxel_grid = np.zeros(voxel_counts)
    print(np.shape(voxel_grid), 'grid shape')
    # Determine which voxels are occupied by points
    for point in intersect_points:
        # Calculate the index of the voxel that contains the point
        voxel_index = np.floor((point - min_coords) / resolution).astype(int)

        # Mark the voxel as occupied
        voxel_grid[tuple(voxel_index)] = 1

    return voxel_grid
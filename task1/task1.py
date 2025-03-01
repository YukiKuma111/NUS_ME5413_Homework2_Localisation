import numpy as np
import open3d as o3d
import datetime
import matplotlib.pyplot as plt
from scipy.spatial import KDTree


def icp_core(point_cloud1, point_cloud2):
    """
    Solve transformation from point_cloud2 to point_cloud1, T1_2
    :param point_cloud1: numpy array, size = n x 3, n is num of point
    :param point_cloud2: numpy array, size = n x 3, n is num of point
    :return: transformation matrix T, size = 4x4

    Note: point cloud should be in same size. Point with same index are corresponding points.
          For example, point_cloud1[i] and point_cloud2[i] are a pair of cooresponding points.

    """
    assert point_cloud1.shape == point_cloud2.shape, "point cloud size not match"

    T1_2 = np.eye(4)
    # TODO: Finish icp based on SVD, you can refer the lecture slides. Please leave comments and explainations for each step.

    save_intermediate_result = False

    # 1. Calculate the centroid of both point clouds
    centroid_pc1 = np.mean(point_cloud1, axis=0)
    centroid_pc2 = np.mean(point_cloud2, axis=0)

    # 2. Center the point clouds by subtracting the centroids
    pc1_centered = point_cloud1 - centroid_pc1
    pc2_centered = point_cloud2 - centroid_pc2

    # 3. Construct the covariance matrix H (for point_cloud2 → point_cloud1)
    H = np.dot(pc2_centered.T, pc1_centered)

    if save_intermediate_result:
        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(pc1_centered.reshape(-1, 3))
        o3d.io.write_point_cloud("intermediate_output1.ply", pcd1)
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(pc2_centered.reshape(-1, 3))
        o3d.io.write_point_cloud("intermediate_output2.ply", pcd2)

    # 4. Perform Singular Value Decomposition (SVD) on the covariance matrix
    U, S, Vt = np.linalg.svd(H)

    # 5. Handle reflection case: if the determinant of R is negative, correct R
    R = np.dot(Vt.T, U.T)

    # Handle singular cases: if det(R) < 0, correct R
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # 6. Calculate the translation vector t
    t = centroid_pc1 - np.dot(R, centroid_pc2)

    # 7. Build the final transformation matrix
    T1_2[0:3, 0:3] = R
    T1_2[0:3, 3] = t.T

    return T1_2


def transform_point_cloud(point_cloud, tfmatrix):
    """
    Apply the transformation matrix to the point cloud.
    :param point_cloud: numpy array of shape (n, 3)
    :param tfmatrix: 4x4 transformation matrix
    :return: transformed point cloud
    """
    # Convert to homogeneous coordinates for matrix multiplication
    pc_homogeneous = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))
    tf_pc_homogeneous = np.dot(tfmatrix, pc_homogeneous.T).T
    tf_point_cloud = tf_pc_homogeneous[:, :3]  # Remove the homogeneous coordinate

    return tf_point_cloud


def solve_icp_with_known_correspondence(point_cloud1, point_cloud2):
    """
    Solve ICP with known correspondences.
    """
    # Solve for transformation matrix
    T1_2 = icp_core(point_cloud1, point_cloud2)
    print("------------ transformation matrix T1_2 ------------")
    print(T1_2)

    start_time = datetime.datetime.now()

    # TODO: calculate transformed point_cloud2 based on T1_2 solved above
    point_cloud2_transformed = transform_point_cloud(point_cloud2, T1_2)

    end_time = datetime.datetime.now()
    time_cost = (end_time - start_time).total_seconds()
    print("time cost = " + str(time_cost) + "s")

    # Visualization
    mean_distance = mean_dist(point_cloud2_transformed, point_cloud1)
    print("mean_error= " + str(mean_distance))
    print("Red for reference cloud")
    print("Green for original cloud")
    print("Blue for transformed cloud")
    print("Red and blue should overlap")

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=10, origin=[0, 0, 0]
    )

    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(point_cloud2)
    pcd2_transformed = o3d.geometry.PointCloud()
    pcd2_transformed.points = o3d.utility.Vector3dVector(point_cloud2_transformed)

    # Color coding for visualization
    pcd1.paint_uniform_color([1, 0, 0])  # Red for reference cloud
    pcd2.paint_uniform_color([0, 1, 0])  # Green for original cloud
    pcd2_transformed.paint_uniform_color([0, 0, 1])  # Blue for transformed cloud

    # Display point clouds
    o3d.visualization.draw_geometries([pcd1, pcd2, pcd2_transformed, axis_pcd])


def estimate_correspondence(point_cloud1, point_cloud2):
    """
    Estimate closest point correspondences between two point clouds using KD-Tree.
    """
    tree = KDTree(point_cloud2)
    _, indices = tree.query(point_cloud1, k=1)
    point_cloud2_reordered = point_cloud2[indices.flatten()]

    save_intermediate_result = False
    if save_intermediate_result:
        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(point_cloud2_reordered)
        o3d.io.write_point_cloud("point_cloud2_reordered.ply", pcd1)

    return point_cloud2_reordered


def solve_icp_without_known_correspondence(
    point_cloud1, point_cloud2, n_iter, threshold
):
    """
    Solve ICP without known correspondences by iteratively refining correspondences.
    """
    point_cloud2_temp = point_cloud2.copy()
    T_1_2accumulated = np.eye(4)
    mean_errors = []

    # viz
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=10, origin=[0, 0, 0]
    )
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd1.paint_uniform_color([0, 0, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(axis_pcd)
    vis.add_geometry(pcd1)

    total_time_cost = 0

    # ICP Iterations
    for i in range(n_iter):
        start_time = datetime.datetime.now()

        # TODO: Try to estimate correspondence of points between 2 point clouds,
        #       and reindex point_cloud2 based on your estimated correspondence
        # 1. Estimate correspondences using KD-Tree
        point_cloud2_reordered = estimate_correspondence(
            point_cloud1, point_cloud2_temp
        )

        # 2. Solve ICP for the current iteration
        T1_2_cur = icp_core(point_cloud1, point_cloud2_reordered)

        end_time = datetime.datetime.now()
        time_difference = (end_time - start_time).total_seconds()
        total_time_cost += time_difference

        # TODO: Update accumulated transformation
        # 3. Accumulate transformations
        T_1_2accumulated = np.dot(T1_2_cur, T_1_2accumulated)

        print("-----------------------------------------")
        print("iteration = " + str(i + 1))
        print("time cost = " + str(time_difference) + "s")
        print("total time cost = " + str(total_time_cost) + "s")
        print("T1_2_cur = ")
        print(T1_2_cur)
        print("accumulated T = ")
        print(T_1_2accumulated)

        # TODO: Update point cloud2 using transform from current iteration
        # 4. Update the point cloud with the new transformation
        point_cloud2_temp = transform_point_cloud(point_cloud2, T_1_2accumulated)

        # 5. Calculate mean distance between the point clouds
        mean_distance = mean_dist(point_cloud1, point_cloud2_temp)
        print("mean_error= " + str(mean_distance))
        mean_errors.append(mean_distance)

        # 6. Update visualization
        pcd2_transed = o3d.geometry.PointCloud()
        pcd2_transed.points = o3d.utility.Vector3dVector(point_cloud2_temp)
        pcd2_transed.paint_uniform_color([1, 0, 0])
        vis.add_geometry(pcd2_transed)
        vis.poll_events()
        vis.update_renderer()
        vis.remove_geometry(pcd2_transed)

        # 7. Check convergence
        if mean_distance < 0.00001 or mean_distance < threshold:
            print("------- fully converged! -------")
            break

        if i == n_iter - 1:
            print("------- reach iteration limit -------")

    plt.figure()
    plt.plot(range(1, len(mean_errors) + 1), mean_errors, marker='.')
    plt.xlabel('Iteration')
    plt.ylabel('Mean Error')
    plt.title('ICP Convergence Analysis')
    plt.grid()
    plt.savefig(f"ICP_Convergence_Analysis_in_{n_iter}_iteration.png")
    plt.show()
    
    print("time cost: " + str(total_time_cost) + " s")

    vis.destroy_window()

    # Final visualization
    pcd2_final = o3d.geometry.PointCloud()
    pcd2_final.points = o3d.utility.Vector3dVector(point_cloud2_temp)
    pcd2_final.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([axis_pcd, pcd1, pcd2_final])


def mean_dist(point_cloud1, point_cloud2):
    """
    Calculate mean Euclidean distance between two point clouds.
    """
    dis_array = []
    for i in range(point_cloud1.shape[0]):
        dif = point_cloud1[i] - point_cloud2[i]
        dis = np.linalg.norm(dif)
        dis_array.append(dis)

    return np.mean(np.array(dis_array))


def main():
    print("start hw program")
    pcd1 = o3d.io.read_point_cloud(
        "student_data/student_data_65/bunny1.ply"
    )  # change to your file path
    pcd2 = o3d.io.read_point_cloud(
        "student_data/student_data_65/bunny2.ply"
    )  # change to your file path
    points1 = np.array(pcd1.points)
    points2 = np.array(pcd2.points)

    # uncomment the lines following task 1 or 2 to run the corresponding task
    # task 1:
    # solve_icp_with_known_correspondence(points1, points2)
    # task 2:
    solve_icp_without_known_correspondence(points1, points2, n_iter=50, threshold=0.1)
    # solve_icp_without_known_correspondence(points1, points2, n_iter=100, threshold=0.1)
    # solve_icp_without_known_correspondence(points1, points2, n_iter=200, threshold=0.01)


if __name__ == "__main__":
    main()

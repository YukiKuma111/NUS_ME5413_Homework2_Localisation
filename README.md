# NUS_ME5413_Homework2_Localisation

## Task 1

### Introduction

The solution for Task 1 of the ME5413: Autonomous Mobile Robot course, focuses on __Point Cloud Registration__ using the __Iterative Closest Point (ICP)__ algorithm. The primary objective is to implement both known and unknown correspondence versions of ICP to align two 3D point clouds.

### Objective

The aim of Task 1 is to implement the ICP algorithm in Python to achieve accurate point cloud registration. The task is divided into two parts:

1. __Task 1.1__ - Implement ICP with known correspondence using SVD-based alignment.

2. __Task 1.2__ - Extend the ICP algorithm to handle unknown correspondences and iteratively refine the alignment until convergence.

### Folder Structure
```
NUS_ME5413_Homework2_Localisation/task1
├── student_data
│   └── student_data_65     # Sample point cloud files used for testing and validation.
│       ├── bunny1.ply
│       └── bunny2.ply
├── task1_ori.py
└── task1.py                # Python script containing the implementation of the ICP algorithm.
```

### How to Run

#### 1. With Known Correspondence:

```
python task1.py
```
This will run the `solve_icp_with_known_correspondence` function and display the results.
![solve_icp_with_known_correspondence result](./img/task1.png)

#### 2. Without Known Correspondence:

Uncomment the corresponding lines in `main()`:
```
# Comment line 145
# solve_icp_with_known_correspondence(points1, points2)
# Uncomment line 147
solve_icp_without_known_correspondence(points1, points2, n_iter=30, threshold=0.1)
```

Then run:
```
python task1.py
```
This will run the `solve_icp_without_known_correspondence` function and display the results.
![solve_icp_without_known_correspondence result](./img/task2_100_0.1.png)

### Task Description
#### Task 1.1: ICP with Known Correspondence

 - __Objective:__ Implement the SVD-based ICP algorithm assuming point correspondences are known.

 - __Steps:__
    1. Compute the centroids of the two point clouds.
    2. Subtract the centroids from the points to center the clouds.
    3. Compute the covariance matrix and perform Singular Value Decomposition (SVD).
    4. Calculate the rotation and translation that aligns the point clouds.
    5. Apply the transformation and visualize the results.

#### Task 1.2: ICP without Known Correspondence

 - __Objective:__ Implement ICP where point correspondences are unknown and iteratively refine alignment.
 - __Steps:__
    1. For each iteration:
         - Find nearest neighbors to establish correspondences.
         - Use the known correspondence ICP method to compute the transformation.
         - Apply the transformation and update the point cloud.
         - Compute the mean error and check for convergence.
    2. Terminate upon reaching convergence criteria or after a set number of iterations.

### Results

For both parts:

- __Quantitative:__
     - Final accumulated transformation matrix.
     - Mean error between point clouds.
     - Total time cost.

 - __Qualitative:__
     - Visualization of initial, intermediate, and final point clouds.
     - Demonstration of the iterative process for unknown correspondence.

### Observations

The __ICP with known correspondence__ provides fast and accurate alignment since the correspondence is explicitly given.
The __ICP without known correspondence__ highlights the importance of good initial positioning and may require more iterations to converge.

### Conclusion

Through this task, the complexities of point cloud registration using the ICP method were explored. While known correspondence simplifies the problem, real-world applications often require iterative refinement due to unknown correspondences. This task also demonstrated the significance of convergence thresholds and iteration limits for achieving precise alignment.

## Task 2
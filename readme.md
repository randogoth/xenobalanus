# Xenobalanus

A Rust implementation of the DELFIN and DTSCAN algorithms to find cluster and void areas in planar point distributions.

## Overview

DELFIN was designed to identify void areas within a point set by leveraging Delaunay triangulation. Void areas are low-density zones within a planar point distribution that have significantly fewer points than their surroundings, ideally none.

The original DELFIN algorithm was developed by Hervías, C., Hitschfeld-Kahler, N., Campusano, L.E., Font, G. (2014). "[On Finding Large Polygonal Voids Using Delaunay Triangulation: The Case of Planar Point Sets](https://doi.org/10.1007/978-3-319-02335-9_16)." In: Sarrate, J., Staten, M. (eds) Proceedings of the 22nd International Meshing Roundtable. Springer, Cham.

DTSCAN is a clustering algorithm that combines Delaunay Triangulation with density-based spatial clustering of applications with noise (DBSCAN) principles to identify clusters in spatial data. Unlike traditional DBSCAN, DTSCAN leverages the geometric properties of Delaunay Triangulation to efficiently manage spatial relationships, making it particularly suited for large datasets and higher-dimensional data. 

DTSCAN was developed by Kim, Jongwon, and Jeongho Cho. "[Delaunay triangulation-based spatial clustering technique for enhanced adjacent boundary detection and segmentation of LiDAR 3D point clouds.](https://doi.org/10.3390%2Fs19183926)" Sensors 19.18 (2019): 3926. in their research paper 'Delaunay Triangulation-Based Spatial Clustering Technique for Enhanced Adjacent Boundary Detection and Segmentation of LiDAR 3D Point Clouds'

## Functions

The combined Xenobalanus implementation is comprised of several key functions:

- `random_points`: Generates uniformly distributed random points for testing.
- `delaunay`: A wrapper of the [Delaunator crate](https://docs.rs/delaunator/latest/delaunator/). Performs Delaunay Triangulation on a given set of points to find their triangular connections.
- `preprocess`: Iterates through all Delaunay triangles to build lookup tables for the DELFIN and DTSCAN functions.
- `delfin`: Processes the lookup tables to find and delineate void areas, based on a threshold that determines what constitutes a significant void.
- `dtscan`: Identifies clusters within the set of points based on the lookup tables, applying a modified DBSCAN algorithm that uses the triangular connections as a basis for neighborhood determination.

## Example Code

Below is an example code snippet that demonstrates the workflow. This example generates random points, runs Delaunay Triangulation on these points, processes the triangulation result, and then performs a cluster and void search.

```rust
use geo::Point;
use std::collections::{HashSet};
use xenobalanus::{delaunay, random_points, preprocess, dtscan, delfin, GeometryData};

fn main() {
    // Define test area and random points
    let dots: u32 = 10000;
    let side_length: f32 = 10000.0;
    let points: Vec<Point<f32>> = random_points((0.0, 0.0), side_length, dots);
    println!("Generated {:#?} random dots", dots);

    // Run Delaunay triangulation
    let triangles_indices: Vec<usize> = delaunay(&points);
    println!("Generated Delaunay triangulation");

    // Pre-process triangles
    let geometry_data: GeometryData = preprocess(&points, &triangles_indices, 0);

    // Execute delfin function with the generated GeometryData
    let min_area: f32 = 1000.0; // threshold for voidness
    let min_distance: f32 = 200.0; // threshold for minimum distance
    let void_polygons: Vec<HashSet<usize>> = delfin(&geometry_data, min_area, min_distance);
    println!("Found {:#?} Voids", void_polygons.len());

    // Execute DTSCAN with the prepared data
    let min_pts: usize = 5; // threshold for minimum number of points
    let max_closeness: f32 = 100.5; // threshold for maximum closeness
    let clusters: Vec<Vec<usize>> = dtscan(&geometry_data, min_pts, max_closeness);
    println!("Found {:#?} Attractors", clusters.len());
}

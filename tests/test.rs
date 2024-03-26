use geo::Point;
use std::collections::{HashSet};
use xenobalanus::{delaunay, random_points, preprocess, dtscan, delfin, GeometryData};

#[test]
fn test() {
    let dots: u32 = 10000;
    let side_length: f32 = 10000.0;
    let points: Vec<Point<f32>> = random_points((0.0, 0.0), side_length, dots);
    println!("Generated {:#?} random dots", dots);
    let triangles_indices: Vec<usize> = delaunay(&points);
    println!("Generated Delaunay triangulation");

    let geometry_data: GeometryData = preprocess(&points, &triangles_indices, 0);

    // Define minimum area and minimum distance for delfin function
    let min_area: f32 = 1000.0; // threshold for voidness
    let min_distance: f32 = 200.0; // threshold for minimum distance

    // Parameters for DTSCAN
    let min_pts: usize = 5; // threshold for minimum number of points
    let max_closeness: f32 = 100.5; // threshold for maximum closeness

    // Execute delfin function with the generated GeometryData
    let void_polygons: Vec<HashSet<usize>> = delfin(&geometry_data, min_area, min_distance);
    println!("Found {:#?} Voids", void_polygons.len());

    // Execute DTSCAN with the prepared data
    let clusters: Vec<Vec<usize>> = dtscan(&geometry_data, min_pts, max_closeness);
    println!("Found {:#?} Attractors", clusters.len());
}
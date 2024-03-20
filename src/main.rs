use geo::{Point, Polygon, LineString, EuclideanDistance, Area};
use std::collections::{HashMap, HashSet};
use std::cmp::{min, max};
use rand::Rng;
use rayon::prelude::*;
use delaunator::{triangulate, Point as DelaunatorPoint};
use itertools::Itertools;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use std::mem;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct Edge(usize, usize);

#[derive(Debug)]
struct TriangleData {
    index: usize,
    area: Option<f32>,
    terminal_edge: Option<Edge>
}

#[derive(Debug)]
struct GeometryData {
    triangles: Vec<TriangleData>,
    edge_to_triangles: HashMap<Edge, Vec<usize>>, // Maps an edge to triangle indices
    edge_lengths: HashMap<Edge, f32>, // Edge lengths
    vertex_connections: HashMap<usize, HashSet<usize>>, // Direct connections between vertices, for DTSCAN
}

impl GeometryData {
    fn new() -> Self {
        GeometryData {
            triangles: Vec::new(),
            edge_to_triangles: HashMap::new(),
            edge_lengths: HashMap::new(),
            vertex_connections: HashMap::new(), // Adjusted for DTSCAN
        }
    }

    fn add_triangle(&mut self, index: usize, points: &[Point<f32>], tri_idx: &[usize], types: usize) {
        let point_a: Point<f32> = points[tri_idx[0]];
        let point_b: Point<f32> = points[tri_idx[1]];
        let point_c: Point<f32> = points[tri_idx[2]];

        // Temporarily store edges_with_lengths for sorting and determining the terminal_edge.
        let mut edges_with_lengths_temp = [
            (Edge(min(tri_idx[0], tri_idx[1]), max(tri_idx[0], tri_idx[1])), point_a.euclidean_distance(&point_b)),
            (Edge(min(tri_idx[1], tri_idx[2]), max(tri_idx[1], tri_idx[2])), point_b.euclidean_distance(&point_c)),
            (Edge(min(tri_idx[2], tri_idx[0]), max(tri_idx[2], tri_idx[0])), point_c.euclidean_distance(&point_a)),
        ].to_vec();
        
        // Sort edges by length to ensure the longest edge is identified.
        edges_with_lengths_temp.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        let terminal_edge: Option<Edge> = edges_with_lengths_temp.first().map(|(edge, _)| *edge);

        let area: Option<f32> = if types == 0 || types == 2 {
            Some(Polygon::new(LineString::from(vec![
                (point_a.x(), point_a.y()),
                (point_b.x(), point_b.y()),
                (point_c.x(), point_c.y()),
                (point_a.x(), point_a.y()),
            ]), vec![]).unsigned_area())
        } else {
            None
        };
    
        if types == 0 || types == 1 {
            for &(edge, length) in &edges_with_lengths_temp {
                self.vertex_connections.entry(edge.0).or_insert_with(HashSet::new).insert(edge.1);
                self.vertex_connections.entry(edge.1).or_insert_with(HashSet::new).insert(edge.0);
                self.edge_lengths.insert(edge, length);
                self.edge_to_triangles.entry(edge).or_default().push(index);
            }
        } else {
            // For types == 2, only update edge_lengths and edge_to_triangles.
            for &(edge, length) in &edges_with_lengths_temp {
                self.edge_lengths.insert(edge, length);
                self.edge_to_triangles.entry(edge).or_default().push(index);
            }
        }
    
        if types == 0 || types == 2 {
            self.triangles.push(TriangleData {
                index,
                area,
                terminal_edge
            });
        }
    }    
}

pub fn random_points(center: (f32, f32), radius: f32, num_points: usize) -> Vec<Point<f32>> {
    let mut rng: rand::prelude::ThreadRng = rand::thread_rng();
    let mut points: Vec<Point<f32>> = Vec::with_capacity(num_points);

    for _ in 0..num_points {
        // Generate a random angle between 0 and 2*PI.
        let angle: f32 = rng.gen_range(0.0..(2.0 * std::f32::consts::PI));
        // Generate a random radius to ensure uniform distribution within the circle.
        let r: f32 = (rng.gen_range(0.0..=1.0) as f32).sqrt() * radius;
        // Calculate x and y coordinates based on the random angle and radius.
        let x: f32 = center.0 + r * angle.cos();
        let y: f32 = center.1 + r * angle.sin();
        // Add the generated point to the points vector.
        points.push(Point::new(x, y));
    }

    points
}

pub fn delaunay(points: &Vec<Point<f32>>) -> Vec<usize> {
    // Convert geo::Point<f32> to delaunator::Point for triangulation
    let delaunator_points: Vec<DelaunatorPoint> = points.iter()
        .map(|point: &Point<f32>| DelaunatorPoint { x: point.x() as f64, y: point.y() as f64 })
        .collect();

    // Perform Delaunay triangulation
    let result: delaunator::Triangulation = triangulate(&delaunator_points);

    // Return the indices of points in the triangles
    result.triangles
}

fn preprocess(points: &[Point<f32>], triangles: &[usize], types: usize) -> GeometryData {
    let geometry_data = Arc::new(Mutex::new(GeometryData::new()));

    triangles.par_chunks(3).enumerate().for_each(|(index, tri_idx)| {
        let gd = geometry_data.clone(); // Clone Arc for use in each thread

        gd.lock().unwrap().add_triangle(index, points, tri_idx, types);
    });

    Arc::try_unwrap(geometry_data).unwrap().into_inner().unwrap()
}

fn delfin(
    geometry_data: &GeometryData,
    min_area: f32,
    min_distance: f32,
) -> Vec<HashSet<usize>> {

    // Sort all triangles by the longest terminal edge
    let triangles_sorted: Vec<(usize, f32)> = geometry_data.triangles.iter()
    .filter_map(|triangle_data| {
        // Only consider triangles with a terminal edge
        triangle_data.terminal_edge.map(|terminal_edge| {
            // Retrieve the length of the terminal edge if it exists
            geometry_data.edge_lengths.get(&terminal_edge)
                .map(|&length| (triangle_data.index, length))
        }).flatten()
    })
    .sorted_by(|a, b| b.1.partial_cmp(&a.1).unwrap()) // Sort in descending order by edge length
    .collect();

    let mut void_polygons: Vec<HashSet<usize>> = Vec::new();
    let mut processed_triangles: HashSet<usize> = HashSet::new();

    for &(triangle_index, terminal_edge_length) in &triangles_sorted {
        // Skip if this triangle has already been processed
        if processed_triangles.contains(&triangle_index) {
            continue;
        }        
    
        // Continue if the terminal edge length is below the minimum distance threshold
        if terminal_edge_length < min_distance {
            continue;
        }

        // Retrieve triangles that share the terminal edge, continue if less than 2 triangles share it
        let triangle_data: &TriangleData = &geometry_data.triangles[triangle_index];
        if let Some(terminal_edge) = triangle_data.terminal_edge {
            if let Some(connected_triangles) = geometry_data.edge_to_triangles.get(&terminal_edge) {
                // Proceed only if there are 2 or more triangles sharing the terminal edge
                if connected_triangles.len() < 2 {
                    continue;
                }
    
                // Initialize the set with the current triangle and triangles directly connected via their terminal edge
                let mut triangle_set: HashSet<usize> = connected_triangles.iter().cloned().collect();
                triangle_set.insert(triangle_index);
                processed_triangles.extend(&triangle_set);
        
                // Dynamically expand the set based on the terminal edge sharing criterion
                let mut triangles_to_expand: HashSet<usize> = triangle_set.clone();
                while let Some(current_idx) = triangles_to_expand.iter().next().cloned() {
                    // Remove the current triangle index from the set to avoid reprocessing
                    triangles_to_expand.remove(&current_idx);
                
                    // Iterate over each triangle that shares a terminal edge
                    for &neighbor_idx in connected_triangles {
                        // Skip if this triangle has already been considered or processed
                        if triangle_set.contains(&neighbor_idx) || processed_triangles.contains(&neighbor_idx) {
                            continue;
                        }
                
                        // Safely access the neighbor triangle's data using its index
                        if let Some(neighbor_data) = geometry_data.triangles.get(neighbor_idx) {
                            // Check if the neighbor shares the same terminal edge
                            // Directly compare the terminal edges as they are both Option<Edge>
                            if neighbor_data.terminal_edge == Some(terminal_edge) {
                                // If they share the same terminal edge, include the neighbor in the current void polygon set
                                triangle_set.insert(neighbor_idx);
                                processed_triangles.insert(neighbor_idx);
                                triangles_to_expand.insert(neighbor_idx);
                            }
                        }
                    }
                }
    
                // Add the expanded set to void polygons
                void_polygons.push(triangle_set);
            } else {
                // If no connected triangles are found for the terminal edge, simply skip to the next triangle
                continue;
            }
        }
    }

    // Filter out void polygon sets
    void_polygons.retain(|poly_set: &HashSet<usize>| {
        // Calculate the total area of the polygon set by summing the areas of the triangles it contains.
        let total_area: f32 = poly_set.iter()
            .filter_map(|&idx| geometry_data.triangles.get(idx).and_then(|td| td.area))
            .sum();
        
        // Filter based on the area and the minimum number of triangles.
        total_area >= min_area && poly_set.len() >= 3
    });

    return void_polygons;

}

fn dtscan(
    geometry_data: &GeometryData,
    min_pts: usize,
    max_closeness: f32,
) -> Vec<Vec<usize>> {
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    let mut visited: HashSet<usize> = HashSet::new();

    for (&vertex_idx, neighbors) in &geometry_data.vertex_connections {
        if visited.contains(&vertex_idx) {
            continue;
        }
        // Check if vertex is a core vertex based on the number of connections and edge lengths
        if neighbors.len() >= min_pts && neighbors.iter().all(|&n| {
            if let Some(&length) = geometry_data.edge_lengths.get(&Edge(min(vertex_idx, n), max(vertex_idx, n))) {
                length <= max_closeness
            } else {
                false
            }
        }) {
            let mut cluster: Vec<usize> = Vec::new();
            let mut to_expand: Vec<usize> = vec![vertex_idx];

            while let Some(current_vertex) = to_expand.pop() {
                if !visited.insert(current_vertex) {
                    continue;
                }

                cluster.push(current_vertex);

                // Add neighbors that are within max_closeness to to_expand
                geometry_data.vertex_connections.get(&current_vertex).map(|neighbors: &HashSet<usize>| {
                    for &neighbor in neighbors {
                        if let Some(&length) = geometry_data.edge_lengths.get(&Edge(min(current_vertex, neighbor), max(current_vertex, neighbor))) {
                            if length <= max_closeness && !visited.contains(&neighbor) {
                                to_expand.push(neighbor);
                            }
                        }
                    }
                });
            }

            if !cluster.is_empty() {
                clusters.push(cluster); // Add the constructed cluster to the list of clusters
            }
        }
    }

    clusters
}

fn main() {
    let dots: usize = 225424;
    let radius: f32 = 1000.0;
    let mut start = Instant::now();
    let points: Vec<Point<f32>> = random_points((0.0, 0.0), radius, dots);
    let mut duration = start.elapsed();
    println!("Generated {:#?} random dots in: {:#?}", dots, duration);
    start = Instant::now();
    let triangles_indices: Vec<usize> = delaunay(&points);
    duration = start.elapsed();
    println!("Generated Delaunay triangulation in: {:#?}", duration);

    start = Instant::now();
    let geometry_data: GeometryData = preprocess(&points, &triangles_indices, 0);
    duration = start.elapsed();
    println!("Preprocessed Triangles using {:#?} bytes of RAM in: {:#?}", mem::size_of_val(&geometry_data), duration);

    // Define minimum area and minimum distance for delfin function
    let min_area: f32 = 75.0; // threshold for voidness
    let min_distance: f32 = 10.0; // threshold for minimum distance

    // Parameters for DTSCAN
    let min_pts: usize = 5; // threshold for minimum number of points
    let max_closeness: f32 = 1.5; // threshold for maximum closeness

    // Execute delfin function with the generated GeometryData
    start = Instant::now();
    let void_polygons: Vec<HashSet<usize>> = delfin(&geometry_data, min_area, min_distance);
    duration = start.elapsed();
    println!("Found {:#?} Voids using {:#?} bytes of RAM in: {:#?}", void_polygons.len(), mem::size_of_val(&void_polygons), duration);

    // Execute DTSCAN with the prepared data
    start = Instant::now();
    let clusters: Vec<Vec<usize>> = dtscan(&geometry_data, min_pts, max_closeness);
    duration = start.elapsed();
    println!("Found {:#?} Attractors using {:#?} bytes of RAM in: {:#?}", clusters.len(), mem::size_of_val(&clusters), duration);
    // println!("{:#?}", (void_polygons, clusters))
}
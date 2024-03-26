use delaunator::{triangulate, Point as DelaunatorPoint};
use geo::{Point, Polygon, LineString, Area};
use itertools::Itertools;
use rand::Rng;
use rayon::prelude::*;
use std::cmp::{min, max};
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct Edge(usize, usize);

#[derive(Debug)]
pub struct TriangleData {
    pub index: usize,
    pub area: Option<f32>,
    pub terminal_edge: Option<Edge>,
    pub vertices: Vec<usize>
}

#[derive(Debug)]
pub struct GeometryData {
    pub triangles: Vec<TriangleData>,
    pub edge_to_triangles: HashMap<Edge, Vec<usize>>, // Maps an edge to triangle indices
    pub edge_lengths: HashMap<Edge, f32>, // Edge lengths
    pub vertex_connections: HashMap<usize, HashSet<usize>>, // Direct connections between vertices, for DTSCAN
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

        let mut vertices = vec![tri_idx[0], tri_idx[1], tri_idx[2]];
        vertices.sort_unstable();

        // Temporarily store edges_with_lengths for sorting and determining the terminal_edge.
        let mut edges_with_lengths_temp = [
            (Edge(min(tri_idx[0], tri_idx[1]), max(tri_idx[0], tri_idx[1])), distance(point_a.x(), point_a.y(), point_b.x(), point_b.y())),
            (Edge(min(tri_idx[1], tri_idx[2]), max(tri_idx[1], tri_idx[2])), distance(point_b.x(), point_b.y(), point_c.x(), point_c.y())),
            (Edge(min(tri_idx[2], tri_idx[0]), max(tri_idx[2], tri_idx[0])), distance(point_c.x(), point_c.y(), point_a.x(), point_a.y())),
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
                terminal_edge,
                vertices
            });
        }
    }    
}

fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt()
}

pub fn random_points(center: (f32, f32), side_length: f32, num_points: u32) -> Vec<Point<f32>> {
    // generate random points in a square
    let min_x = center.0 - side_length / 2.0;
    let max_x = center.0 + side_length / 2.0;
    let min_y = center.1 - side_length / 2.0;
    let max_y = center.1 + side_length / 2.0;
    let mut points: Vec<Point<f32>> = Vec::with_capacity(num_points as usize);
    let mut rng: rand::prelude::ThreadRng = rand::thread_rng();
    for _ in 0..num_points {
        let x = min_x + rng.gen_range(0.0..=1.0) as f32 * ( max_x - min_x);
        let y: f32 = min_y + rng.gen_range(0.0..=1.0) as f32 * ( max_y - min_y);
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

pub fn preprocess(points: &[Point<f32>], triangles: &[usize], types: usize) -> GeometryData {
    let geometry_data = Arc::new(Mutex::new(GeometryData::new()));

    triangles.par_chunks(3).enumerate().for_each(|(index, tri_idx)| {
        let gd = geometry_data.clone(); // Clone Arc for use in each thread

        gd.lock().unwrap().add_triangle(index, points, tri_idx, types);
    });

    Arc::try_unwrap(geometry_data).unwrap().into_inner().unwrap()
}

pub fn delfin(
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

pub fn dtscan(
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
use geo::{Point, Polygon, LineString, EuclideanDistance, Area};
use std::collections::{HashMap, HashSet};
use std::cmp::{min, max};
use rand::Rng;
use rayon::prelude::*;
use delaunator::{triangulate, Point as DelaunatorPoint};
use itertools::Itertools;
use std::sync::{Arc, Mutex};


#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct Edge(usize, usize);

#[derive(Debug)]
struct TriangleData {
    index: usize,
    area: Option<f32>,
    terminal_edge: Option<Edge>,
    edges_with_lengths: Option<Vec<(Edge, f32)>>,
    node_connections: Option<HashSet<usize>>,
}

#[derive(Debug)]
struct GeometryData {
    triangles: Vec<TriangleData>,
    edge_to_triangles: HashMap<Edge, Vec<usize>>, // Maps an edge to triangle indices
    edge_lengths: HashMap<Edge, f32>, // Edge lengths
    vertex_to_triangles: HashMap<usize, Vec<usize>>, // Maps a vertex to connected triangle indices
}

impl GeometryData {
    fn new() -> Self {
        GeometryData {
            triangles: Vec::new(),
            edge_to_triangles: HashMap::new(),
            edge_lengths: HashMap::new(),
            vertex_to_triangles: HashMap::new(),
        }
    }

    fn add_triangle(&mut self, index: usize, points: &[Point<f32>], tri_idx: &[usize], types: usize) {
        let point_a = points[tri_idx[0]];
        let point_b = points[tri_idx[1]];
        let point_c = points[tri_idx[2]];
    
        let edges_with_lengths: Option<Vec<(Edge, f32)>> = if types == 0 || types == 2 {
            Some([
                (Edge(min(tri_idx[0], tri_idx[1]), max(tri_idx[0], tri_idx[1])), point_a.euclidean_distance(&point_b)),
                (Edge(min(tri_idx[1], tri_idx[2]), max(tri_idx[1], tri_idx[2])), point_b.euclidean_distance(&point_c)),
                (Edge(min(tri_idx[2], tri_idx[0]), max(tri_idx[2], tri_idx[0])), point_c.euclidean_distance(&point_a)),
            ].to_vec().into_iter().sorted_by(|a, b| b.1.partial_cmp(&a.1).unwrap()).collect())
        } else {
            None
        };
    
        let terminal_edge = edges_with_lengths.as_ref().map(|edges| edges[0].0);
        let area = if types == 0 || types == 2 {
            Some(Polygon::new(LineString::from(vec![
                (point_a.x(), point_a.y()),
                (point_b.x(), point_b.y()),
                (point_c.x(), point_c.y()),
                (point_a.x(), point_a.y()),
            ]), vec![]).unsigned_area())
        } else {
            None
        };
    
        let node_connections: Option<HashSet<usize>> = if types == 0 || types == 1 {
            Some(tri_idx.iter().cloned().collect())
        } else {
            None
        };
    
        self.triangles.push(TriangleData {
            index,
            area,
            terminal_edge,
            edges_with_lengths: edges_with_lengths.clone(),
            node_connections: node_connections.clone(),
        });
    
        if let Some(edges) = &edges_with_lengths {
            for &(edge, length) in edges {
                self.edge_lengths.insert(edge, length);
                self.edge_to_triangles.entry(edge).or_default().push(index);
            }
        }
    
        if let Some(nodes) = &node_connections {
            for &vertex in nodes {
                self.vertex_to_triangles.entry(vertex).or_default().push(index);
            }
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
    let geometry_data: Arc<Mutex<GeometryData>> = Arc::new(Mutex::new(GeometryData::new()));

    triangles.par_chunks(6).enumerate().for_each(|(index, tri_idx)| {
        let points_clone: Vec<Point<f32>> = points.to_vec(); // Clone points to avoid borrowing issues
        let gd: Arc<Mutex<GeometryData>> = geometry_data.clone(); // Clone Arc for use in each thread

        gd.lock().unwrap().add_triangle(index, &points_clone, tri_idx, types);
    });

    // Extract the GeometryData from the Arc<Mutex<>>. This is safe to do here because
    // the par_iter has completed, and we know no other threads are accessing it.
    Arc::try_unwrap(geometry_data).unwrap().into_inner().unwrap()
}


fn mean_std(dataset: Vec<f32>) -> (f32, f32) {
    let mean: f32 = dataset.iter().sum::<f32>() / dataset.len() as f32;
    let std: f32 = (dataset.iter().map(|&length| {
        let diff = length - mean;
        diff * diff}
    ).sum::<f32>() / dataset.len() as f32).sqrt();
    (mean, std)
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

    // Calculate areas for triangles that have an area calculated
    let areas: Vec<f32> = geometry_data.triangles.par_iter()
        .filter_map(|triangle_data| triangle_data.area)
        .map(|area| area)
        .collect();

    // Calculate mean and standard deviation of terminal edges lengths
    let terminal_edge_lengths: Vec<f32> = geometry_data.triangles.par_iter()
    .filter_map(|triangle_data| {
        triangle_data.terminal_edge.and_then(|edge| geometry_data.edge_lengths.get(&edge))
    })
    .cloned()
    .collect();

    let (mean_terminal_edge, std_terminal_edge) = mean_std(terminal_edge_lengths);
    let (mean_area, std_area) = mean_std(areas);

    let mut void_polygons: Vec<HashSet<usize>> = Vec::new();
    let mut processed_triangles: HashSet<usize> = HashSet::new();

    for &(triangle_index, terminal_edge_length) in &triangles_sorted {
        // Skip if this triangle has already been processed
        if processed_triangles.contains(&triangle_index) {
            continue;
        }        
    
        // Calculate the Z-score for the terminal edge length
        let distance_z_score: f32 = (terminal_edge_length - mean_terminal_edge) / std_terminal_edge;
        // Continue if the Z-score is below the minimum distance threshold
        if distance_z_score < min_distance {
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
        
        // Calculate the area Z-score if std_area is non-zero to avoid division by zero.
        let area_z_score: f32 = if std_area != 0.0 {
            (total_area - mean_area) / std_area
        } else {
            -5.0
        };
        
        // Filter based on the area Z-score and the minimum number of triangles.
        area_z_score >= min_area && poly_set.len() >= 3
    });
    
    return void_polygons;
}

fn main() {
    let points: Vec<Point<f32>> = random_points((0.0, 0.0), 1000.0, 10000);
    let triangles_indices: Vec<usize> = delaunay(&points);

    // Preprocess to create GeometryData
    let geometry_data: GeometryData = preprocess(&points, &triangles_indices, 0);

    // Define minimum area and minimum distance for delfin function
    let min_area: f32 = 4.0; // Example threshold for voidness
    let min_distance: f32 = 1.0; // Example threshold for minimum distance (Z-score)

    // Execute delfin function with the generated GeometryData
    let void_polygons: Vec<HashSet<usize>> = delfin(&geometry_data, min_area, min_distance);

    // To display the result, let's just print the count of void polygons found
    println!("Void Polygons Found: {:?}", void_polygons.len());
}

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
    area: f32,
    terminal_edge: Edge,
    edges_with_lengths: Vec<(Edge, f32)>,
    node_connections: HashSet<usize>,
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

    // Function to add a triangle to the GeometryData
    fn add_triangle(&mut self, index: usize, points: &[Point<f32>], tri_idx: &[usize]) {
        let point_a: Point<f32> = points[tri_idx[0]];
        let point_b: Point<f32> = points[tri_idx[1]];
        let point_c: Point<f32> = points[tri_idx[2]];

        let edges_with_lengths = [
            (Edge(min(tri_idx[0], tri_idx[1]), max(tri_idx[0], tri_idx[1])), point_a.euclidean_distance(&point_b)),
            (Edge(min(tri_idx[1], tri_idx[2]), max(tri_idx[1], tri_idx[2])), point_b.euclidean_distance(&point_c)),
            (Edge(min(tri_idx[2], tri_idx[0]), max(tri_idx[2], tri_idx[0])), point_c.euclidean_distance(&point_a)),
        ];

        let mut edges_sorted = edges_with_lengths.to_vec();
        edges_sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let terminal_edge = edges_sorted[0].0;
        let area = Polygon::new(LineString::from(vec![
            (point_a.x(), point_a.y()), 
            (point_b.x(), point_b.y()), 
            (point_c.x(), point_c.y()), 
            (point_a.x(), point_a.y())
        ]), vec![]).unsigned_area();

        let node_connections: HashSet<usize> = tri_idx.iter().cloned().collect::<HashSet<_>>();

        self.triangles.push(TriangleData {
            index,
            area,
            terminal_edge,
            edges_with_lengths: edges_sorted,
            node_connections,
        });

        for &(edge, length) in &edges_with_lengths {
            self.edge_lengths.insert(edge, length);
            self.edge_to_triangles.entry(edge).or_default().push(index);
        }

        for &vertex in tri_idx {
            self.vertex_to_triangles.entry(vertex).or_default().push(index);
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

fn preprocess(points: &[Point<f32>], triangles: &[usize]) -> GeometryData {
    let geometry_data: Arc<Mutex<GeometryData>> = Arc::new(Mutex::new(GeometryData::new()));

    triangles.par_chunks(6).enumerate().for_each(|(index, tri_idx)| {
        let points_clone: Vec<Point<f32>> = points.to_vec(); // Clone points to avoid borrowing issues
        let gd: Arc<Mutex<GeometryData>> = geometry_data.clone(); // Clone Arc for use in each thread

        gd.lock().unwrap().add_triangle(index, &points_clone, tri_idx);
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
    .map(|triangle_data: &TriangleData| {
        let terminal_edge_length: f32 = geometry_data.edge_lengths[&triangle_data.terminal_edge];
        (triangle_data.index, terminal_edge_length)
    })
    .sorted_by(|a, b| b.1.partial_cmp(&a.1).unwrap()) // Sort in descending order by edge length
    .collect();

    // Calculate densities based on reverse area
    let areas: Vec<f32> = geometry_data.triangles.par_iter()
    .map(|triangle_data: &TriangleData| triangle_data.area)
    .collect();

    // Calculate mean and standard deviation of terminal edges lengths
    let terminal_edge_lengths: Vec<f32> = geometry_data.triangles.par_iter()
    .map(|triangle_data: &TriangleData| geometry_data.edge_lengths[&triangle_data.terminal_edge])
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
        let terminal_edge: Edge = triangle_data.terminal_edge;
        if let Some(connected_triangles) = geometry_data.edge_to_triangles.get(&terminal_edge) {
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
                triangles_to_expand.remove(&current_idx);
    
                // For each triangle, check its edges against the edges of the neighbors
                for &neighbor_idx in connected_triangles {
                    if triangle_set.contains(&neighbor_idx) || processed_triangles.contains(&neighbor_idx) {
                        continue;
                    }
    
                    let neighbor_data = &geometry_data.triangles[neighbor_idx];
                    // Check if neighbor shares a terminal edge
                    if neighbor_data.terminal_edge == terminal_edge {
                        triangle_set.insert(neighbor_idx);
                        processed_triangles.insert(neighbor_idx);
                        triangles_to_expand.insert(neighbor_idx);
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

    // Filter out void polygon sets
    void_polygons.retain(|poly_set: &HashSet<usize>| {
        // Calculate the total area of the polygon set by summing the areas of the triangles it contains
        let total_area: f32 = poly_set.iter()
            .filter_map(|&idx| geometry_data.triangles.get(idx))
            .map(|triangle_data: &TriangleData| triangle_data.area)
            .sum();
    
        // Calculate the area Z-score
        let area_z_score: f32 = (total_area - mean_area) / std_area;
    
        // Filter based on the area Z-score and the minimum number of triangles
        area_z_score >= min_area && poly_set.len() >= 3
    });
    
    return void_polygons;
}

fn main() {
    let points: Vec<Point<f32>> = random_points((0.0, 0.0), 1000.0, 10000);
    let triangles_indices: Vec<usize> = delaunay(&points);

    // Preprocess to create GeometryData
    let geometry_data: GeometryData = preprocess(&points, &triangles_indices);

    // Define minimum voidness and minimum distance for delfin function
    let min_area: f32 = 4.0; // Example threshold for voidness
    let min_distance: f32 = 1.0; // Example threshold for minimum distance (Z-score)

    // Execute delfin function with the generated GeometryData
    let void_polygons: Vec<HashSet<usize>> = delfin(&geometry_data, min_area, min_distance);

    // To display the result, let's just print the count of void polygons found
    println!("Void Polygons Found: {:?}", void_polygons.len());
}

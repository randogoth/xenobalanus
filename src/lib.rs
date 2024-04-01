use delaunator::{triangulate, Point as DelaunatorPoint};
use geo::{Point as GeoPoint, Coord};
use rand::Rng;
use rayon::prelude::*;
use std::cmp::{min, max};
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub x: f32,
    pub y: f32
}

impl Point {

    pub fn new(x: f32, y: f32) -> Self {
        Point{ x: x, y: y }
    }

    pub fn from_geo32(point: GeoPoint<f32>) -> Self {
        Point{ x: point.x(), y: point.y() }
    }

    pub fn from_geo64(point: GeoPoint<f64>) -> Self {
        Point{ x: point.x() as f32, y: point.y() as f32 }
    }

    pub fn distance(&self,point: Point) -> f32 {
        ( (point.x - &self.x).powi(2) + (point.y - &self.y).powi(2) ).sqrt()
    }

    pub fn bearing(&self, point: Point) -> f32 {
        let delta_x = point.x - self.x;
        let delta_y = point.y - self.y;
        delta_y.atan2(delta_x).to_degrees().rem_euclid(360.0)
    }
}

impl From<Point> for Coord<f32> {
    fn from(point: Point) -> Self {
        Coord { x: point.x, y: point.y }
    }
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct Edge(usize, usize);

#[derive(Debug, Default, Clone)]
pub struct TriangleData {
    pub index: usize,
    pub area: Option<f32>,
    pub terminal_edge: Option<Edge>,
    pub vertices: Vec<usize>
}

impl TriangleData {
    pub fn get_edges(&self) -> Vec<Edge> {
        let mut edges = Vec::new();
        if self.vertices.len() >= 3 {
            for i in 0..self.vertices.len() {
                let v1 = self.vertices[i];
                let v2 = if i + 1 < self.vertices.len() {
                    self.vertices[i + 1]
                } else {
                    self.vertices[0]
                };
                edges.push(if v1 < v2 { Edge(v1, v2) } else { Edge(v2, v1) });
            }
        }
        edges
    }
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
    fn add_triangle(&mut self, index: usize, points: &[Point], tri_idx: &[usize], types: usize) {

        let point_a: Point = points[tri_idx[0]];
        let point_b: Point = points[tri_idx[1]];
        let point_c: Point = points[tri_idx[2]];

        let mut vertices = vec![tri_idx[0], tri_idx[1], tri_idx[2]];
        vertices.sort_unstable();

        // Temporarily store edges_with_lengths for sorting and determining the terminal_edge.
        let mut edges_with_lengths_temp = [
            (Edge(min(tri_idx[0], tri_idx[1]), max(tri_idx[0], tri_idx[1])), point_a.distance(point_b)),
            (Edge(min(tri_idx[1], tri_idx[2]), max(tri_idx[1], tri_idx[2])), point_b.distance(point_c)),
            (Edge(min(tri_idx[2], tri_idx[0]), max(tri_idx[2], tri_idx[0])), point_c.distance(point_a)),
        ].to_vec();
        
        // Sort edges by length to ensure the longest edge is identified.
        edges_with_lengths_temp.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        
        let terminal_edge: Option<Edge> = edges_with_lengths_temp.first().map(|(edge, _)| *edge);
        
        let area: Option<f32> = if types == 0 || types == 2 {
            let x1 = point_a.x;
            let y1 = point_a.y;
            let x2 = point_b.x;
            let y2 = point_b.y;
            let x3 = point_c.x;
            let y3 = point_c.y;
        
            // Calculate the area using the shoelace formula
            let calculated_area = (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2)).abs() / 2.0;
            Some(calculated_area)
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

        // Dynamically resize the struct in memory to accomodate the index
        if index >= self.triangles.len() {
            self.triangles.resize(index + 1, TriangleData::default());
        }
    
        if types == 0 || types == 2 {
            self.triangles[index] = TriangleData {
                index,
                area,
                terminal_edge,
                vertices
            };
        }
    } 
          
}

pub struct Xenobalanus {
    geometry_data: GeometryData,
    points: Vec<Point>,
    triangulation: Vec<usize>,
}

impl Xenobalanus {
    pub fn new() -> Self {
        Xenobalanus {
            geometry_data: GeometryData::new(),
            points: Vec::new(),
            triangulation: Vec::new(),
        }
    }

    pub fn point(&self, index: usize) -> Point {
        self.points[index]
    }

    pub fn points(&self) -> Vec<(f32, f32)> {
        self.points.iter()
            .map(|point| (point.x, point.y))
            .collect()
    }

    pub fn points_flat(&self) -> Vec<f32> {
        self.points.iter()
            .flat_map(|point| vec![point.x, point.y])
            .collect()
    }

    pub fn set_points(&mut self, points: Vec<Point>) {
        self.points = points
    }

    pub fn triangle(&self, index: usize) -> TriangleData {
        self.geometry_data.triangles[index].clone()
    }

    pub fn triangle_data(&self) -> &Vec<TriangleData> {
        &self.geometry_data.triangles
    }

    pub fn triangles_flat(&self) -> Vec<usize> {
        self.triangulation.clone()
    }

    pub fn triangle_vertices(&self) -> Vec<Vec<usize>> {
        self.triangulation.chunks(3).map(|chunk| {
            chunk.iter().map(|&index| index).collect()
        }).collect()
    }

    pub fn triangle_coordinates(&self) -> Vec<Vec<(f32, f32)>> {
        self.triangulation.chunks(3).map(|chunk| {
            chunk.iter().map(|&index| {
                let point = &self.points[index];
                (point.x, point.y) // Each point is represented by a Vec<f32> of its coordinates
            }).collect() // Collects points of a triangle into Vec<Vec<f32>>
        }).collect() // Collects all triangles into Vec<Vec<Vec<f32>>>
    }

    pub fn set_triangles(&mut self, vertices: Vec<usize>) {
        self.triangulation = vertices
    }

    // Additional methods moved into GeometryProcessor, operating on self.geometry_data
    pub fn random_points(&mut self, center: (f32, f32), side_length: f32, num_points: u32) {
        // generate random points in a square
        let min_x = center.0 - side_length / 2.0;
        let max_x = center.0 + side_length / 2.0;
        let min_y = center.1 - side_length / 2.0;
        let max_y = center.1 + side_length / 2.0;
        let mut rng: rand::prelude::ThreadRng = rand::thread_rng();
        for _ in 0..num_points {
            let x = min_x + rng.gen_range(0.0..=1.0) as f32 * ( max_x - min_x);
            let y: f32 = min_y + rng.gen_range(0.0..=1.0) as f32 * ( max_y - min_y);
            self.points.push(Point {x, y});
        }
    }

    pub fn edge_lengths(&self) -> &HashMap<Edge, f32> {
        &self.geometry_data.edge_lengths
    }

    pub fn delaunay(&mut self) {
        // Convert geo::Point to delaunator::Point for triangulation
        let delaunator_points: Vec<DelaunatorPoint> = self.points.iter()
        .map(|point: &Point| DelaunatorPoint { x: point.x as f64, y: point.y as f64 })
        .collect();

    // Perform Delaunay triangulation
    let result: delaunator::Triangulation = triangulate(&delaunator_points);
    self.triangulation = result.triangles
    }

    pub fn preprocess(&mut self, types: usize) {
        let geometry_data = Arc::new(Mutex::new(GeometryData::new()));
    
        self.triangulation.par_chunks(3).enumerate().for_each(|(index, tri_idx)| {
            let gd = geometry_data.clone(); // Clone Arc for use in each thread, not the data itself
    
            // Perform locked update
            let mut gd_lock = gd.lock().unwrap();
            gd_lock.add_triangle(index, &self.points, tri_idx, types);
        });
    
        self.geometry_data = Arc::try_unwrap(geometry_data).unwrap().into_inner().unwrap();
    }

    pub fn delfin(
        &self,
        min_area: f32,
        min_distance: f32,
    ) -> Vec<HashSet<usize>> {
        let mut void_polygons: Vec<HashSet<usize>> = Vec::new();
        let mut processed_triangles: HashSet<usize> = HashSet::new();
    
        // Create a sorted list of triangles by their terminal edge length that meet the minimum distance criteria.
        let mut triangles_sorted: Vec<(usize, f32)> = self.geometry_data.triangles.iter()
            .filter_map(|t| t.terminal_edge.and_then(|e| self.geometry_data.edge_lengths.get(&e).map(|&l| (t.index, l))))
            .filter(|&(_, length)| length >= min_distance)
            .collect();
    
        // Sort by longest edge first
        triangles_sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    
        // Iterate through triangles starting from the one with the longest terminal edge
        for (triangle_index, _) in triangles_sorted {

            // Skip if already processed
            if processed_triangles.contains(&triangle_index) {
                continue;
            }
            
            let mut edges_to_expand: HashSet<Edge> = HashSet::new();
            let mut current_set: HashSet<usize> = HashSet::new();
            
            // Seed the initial set and edges to expand
            current_set.insert(triangle_index);
            processed_triangles.insert(triangle_index);

            // Get all edges of the current triangle
            if let Some(edges) = self.geometry_data.triangles.get(triangle_index).map(|t| t.get_edges()) {
                for edge in edges {
                    
                    // Add all edges to check for neighbors to expand
                    edges_to_expand.insert(edge);
                }
            }
    
            // Expand the set
            while let Some(edge) = edges_to_expand.iter().next().cloned() {
                edges_to_expand.remove(&edge);
    
                // Get neighbor triangles for this edge
                if let Some(neighbor_triangles) = self.geometry_data.edge_to_triangles.get(&edge) {

                    // Iterate through neighbors
                    for &neighbor_index in neighbor_triangles {

                        // Skip if already processed
                        if processed_triangles.contains(&neighbor_index) {
                            continue;
                        }

                        // Get neighbor triangle
                        if let Some(neighbor_triangle) = self.geometry_data.triangles.get(neighbor_index) {
                            
                            // Get neighbor triangle's terminal edge
                            if let Some(neighbor_edge) = neighbor_triangle.terminal_edge {

                                // If neighbor's terminal edge is edge of current triangle, add to set
                                if neighbor_edge == edge {
                                    current_set.insert(neighbor_index);
                                    processed_triangles.insert(triangle_index);
                                    processed_triangles.insert(neighbor_index);

                                    // Add new neighbor edges to search
                                    neighbor_triangle.get_edges().into_iter().for_each(|e| { edges_to_expand.insert(e); });
                                }
                            }
                        }
                    }
                }
            }
            
            // Add the expanded set if more than one triangle
            if current_set.len() > 1 {
                void_polygons.push(current_set);
            }
        }
        
        // Retain only those sets that meet the minimum area criteria
        void_polygons.retain(|set| {
            set.iter()
               .filter_map(|&i| self.geometry_data.triangles[i].area)
               .sum::<f32>() >= min_area
        });
    
        void_polygons
    }    

    pub fn dtscan(
        &self,
        min_pts: usize,
        max_closeness: f32,
    ) -> Vec<Vec<usize>> {
        let mut clusters: Vec<Vec<usize>> = Vec::new();
        let mut visited: HashSet<usize> = HashSet::new();
    
        for (&vertex_idx, neighbors) in &self.geometry_data.vertex_connections {
            if visited.contains(&vertex_idx) {
                continue;
            }
            // Check if vertex is a core vertex based on the number of connections and edge lengths
            if neighbors.len() >= min_pts && neighbors.iter().all(|&n| {
                if let Some(&length) = self.geometry_data.edge_lengths.get(&Edge(min(vertex_idx, n), max(vertex_idx, n))) {
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
                    self.geometry_data.vertex_connections.get(&current_vertex).map(|neighbors: &HashSet<usize>| {
                        for &neighbor in neighbors {
                            if let Some(&length) = self.geometry_data.edge_lengths.get(&Edge(min(current_vertex, neighbor), max(current_vertex, neighbor))) {
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
}

impl Xenobalanus {

    pub fn delaunay_sub(&self, vertices: Vec<usize>) -> Vec<usize> {
        let delaunator_points: Vec<DelaunatorPoint> = vertices.iter()
        .map(|vertex| DelaunatorPoint { x: self.point(*vertex).x as f64, y: self.point(*vertex).y as f64 })
        .collect();

        // Perform Delaunay triangulation
        let result: delaunator::Triangulation = triangulate(&delaunator_points);
        result.triangles

    }

    /// Calculates the concave hull for a subset of vertices indicated by their indices, based on the alpha parameter.
    pub fn concave_hull(&self, vertex_indices: Vec<usize>, alpha: f32) -> Result<Vec<usize>, &'static str> {
        // Perform Delaunay triangulation on the subset of vertices.
        let triangulation_indices = self.delaunay_sub(vertex_indices.clone());

        if triangulation_indices.is_empty() {
            return Err("Delaunay triangulation failed or no triangles were formed.");
        }

        // Initialize edge counter to identify unique edges
        let mut edge_counter: HashMap<(usize, usize), usize> = HashMap::new();

        // Iterate through triangles to populate edge counter
        for chunk in triangulation_indices.chunks(3) {
            if chunk.len() == 3 {
                let global_indices = [vertex_indices[chunk[0]], vertex_indices[chunk[1]], vertex_indices[chunk[2]]];
                
                // Process each edge in the triangle
                for i in 0..3 {
                    let start_idx = global_indices[i];
                    let end_idx = global_indices[(i + 1) % 3];
                    let edge = (start_idx.min(end_idx), start_idx.max(end_idx)); // Ensure consistent ordering

                    // Apply alpha filter based on the distance between points
                    let distance = self.point(start_idx).distance(self.point(end_idx));
                    if distance < alpha {
                        *edge_counter.entry(edge).or_insert(0) += 1;
                    }
                }
            }
        }

        // Extract edges that appear exactly once and are within the alpha radius
        let hull_edge_indices: Vec<(usize, usize)> = edge_counter.into_iter()
            .filter_map(|(edge, count)| if count == 1 { Some(edge) } else { None })
            .collect();

        if hull_edge_indices.is_empty() {
            return Err("No edges meet the criteria for the concave hull.");
        }

        // Order the hull edge indices to form a continuous path
        let ordered_indices = self.order_hull_edges(hull_edge_indices)?;

        Ok(ordered_indices)
    }

    /// Attempts to order hull edges into a continuous path.
    pub fn order_hull_edges(&self, hull_edge_indices: Vec<(usize, usize)>) -> Result<Vec<usize>, &'static str> {
        if hull_edge_indices.is_empty() {
            return Err("No edges provided.");
        }

        let mut visited: HashSet<usize> = HashSet::new();
        let mut ordered_point_indices: Vec<usize> = Vec::new();

        // Initialize with the first edge's indices
        let (start_idx, mut current_idx) = hull_edge_indices[0];
        ordered_point_indices.push(start_idx);
        visited.insert(start_idx);

        while visited.len() < hull_edge_indices.len() + 1 {
            let mut found_next = false;

            for &(p1_idx, p2_idx) in &hull_edge_indices {
                if p1_idx == current_idx && !visited.contains(&p2_idx) {
                    ordered_point_indices.push(p2_idx);
                    visited.insert(p2_idx);
                    current_idx = p2_idx;
                    found_next = true;
                    break;
                } else if p2_idx == current_idx && !visited.contains(&p1_idx) {
                    ordered_point_indices.push(p1_idx);
                    visited.insert(p1_idx);
                    current_idx = p1_idx;
                    found_next = true;
                    break;
                }
            }

            if !found_next {
                return Err("Failed to order all concave hull vertices into a continuous path.");
            }
        }

        // Convert indices to Points
        Ok(hull_edge_indices.iter().map(|&(start_idx, _)| start_idx).collect())

    }
}
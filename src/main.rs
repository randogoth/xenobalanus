use geo::{Point, Polygon, LineString, EuclideanDistance, Area};
use std::collections::{HashMap, HashSet};
use std::cmp::{min, max};
use rand::Rng;
use delaunator::{triangulate, Point as DelaunatorPoint};

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

pub fn delaunay(points: Vec<Point<f32>>) -> Vec<usize> {
    // Convert geo::Point<f32> to delaunator::Point for triangulation
    let delaunator_points: Vec<DelaunatorPoint> = points.iter()
        .map(|point: &Point<f32>| DelaunatorPoint { x: point.x() as f64, y: point.y() as f64 })
        .collect();

    // Perform Delaunay triangulation
    let result: delaunator::Triangulation = triangulate(&delaunator_points);

    // Return the indices of points in the triangles
    result.triangles
}

fn preprocess(dots: Vec<(f32, f32)>, triangles: Vec<usize>) -> (Vec<Point<f32>>, HashMap<usize, HashSet<usize>>, HashMap<usize, f32>, HashMap<(usize, usize), HashSet<usize>>, HashMap<(usize, usize), f32>, HashMap<usize, HashSet<usize>>) {
    let points: Vec<Point<f32>> = dots.into_iter().map(|(x, y)| Point::new(x, y)).collect();

    let mut wing_map: HashMap<(usize, usize), HashSet<usize>> = HashMap::new();
    let mut node_map: HashMap<usize, HashSet<usize>> = HashMap::new();
    let mut edge_map: HashMap<(usize, usize), f32> = HashMap::new();
    let mut terminal_map: HashMap<usize, HashSet<usize>> = HashMap::new();
    let mut area_map: HashMap<usize, f32> = HashMap::new();

    for i in (0..triangles.len()).step_by(3) {
        let tri_idx = &triangles[i..i + 3];
        let point_a = points[tri_idx[0]];
        let point_b = points[tri_idx[1]];
        let point_c = points[tri_idx[2]];

        let edges_with_lengths = [
            ((min(tri_idx[0], tri_idx[1]), max(tri_idx[0], tri_idx[1])), point_a.euclidean_distance(&point_b)),
            ((min(tri_idx[1], tri_idx[2]), max(tri_idx[1], tri_idx[2])), point_b.euclidean_distance(&point_c)),
            ((min(tri_idx[2], tri_idx[0]), max(tri_idx[2], tri_idx[0])), point_c.euclidean_distance(&point_a)),
        ];

        let mut edges_sorted = edges_with_lengths.to_vec();
        edges_sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        terminal_map.insert(i / 3, [edges_sorted[0].0 .0, edges_sorted[0].0 .1].iter().cloned().collect());

        for &(edge, length) in &edges_sorted {
            wing_map.entry(edge).or_insert_with(HashSet::new).insert(i / 3);
            edge_map.insert(edge, length);
        }

        for &idx in tri_idx.iter() {
            node_map.entry(idx).or_insert_with(HashSet::new).extend(tri_idx.iter().filter(|&&x| x != idx).cloned());
        }

        let poly = Polygon::new(LineString::from(vec![
            (point_a.x(), point_a.y()),
            (point_b.x(), point_b.y()),
            (point_c.x(), point_c.y()),
            (point_a.x(), point_a.y()), // Close the loop
        ]), vec![]);

        area_map.insert(i / 3, poly.unsigned_area()); // Correctly calculate the area
    }

    (points, terminal_map, area_map, wing_map, edge_map, node_map)
}


fn main() {
    println!("Hello, world!");
}

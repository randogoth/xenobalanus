use rand::Rng;
use simple_delaunay_lib::delaunay_3d::simplicial_struct_3d::Node;
use std::cmp::{min, max};
use std::collections::HashSet;

use super::{Edge, Xenobalanus};

pub struct Point3D {
    x: f32,
    y: f32,
    z: f32,
}

impl Point3D {

    pub fn new(coords: (f32, f32, f32)) -> Point3D {
        Point3D {x: coords.0, y: coords.1, z: coords.2 }
    }

    pub fn arr(&self) -> [f64; 3] {
        [self.x as f64, self.y as f64, self.z as f64]
    }
 
    pub fn distance(&self,point: &Point3D) -> f32 {
        ( 
            (point.x - &self.x).powi(2) + 
            (point.y - &self.y).powi(2) + 
            (point.z - &self.z).powi(2) 
        ).sqrt()
    }
}

impl Xenobalanus {

    pub fn random_points_3d(&mut self, center: (f32, f32), side_length: f32, num_points: u32) {
        
        // generate random points in a cube
        let min_x = center.0 - side_length / 2.0;
        let max_x = center.0 + side_length / 2.0;
        let min_y = center.1 - side_length / 2.0;
        let max_y = center.1 + side_length / 2.0;
        let min_z = center.0 - side_length / 2.0;
        let max_z = center.0 + side_length / 2.0;
        let mut rng: rand::prelude::ThreadRng = rand::thread_rng();
        for _ in 0..num_points {
            let x = min_x + rng.gen_range(0.0..=1.0) as f32 * ( max_x - min_x);
            let y: f32 = min_y + rng.gen_range(0.0..=1.0) as f32 * ( max_y - min_y);
            let z: f32 = min_y + rng.gen_range(0.0..=1.0) as f32 * ( max_z - min_z);
            self.nodes.push(Point3D {x, y, z});
        }
    }

    pub fn delaunay_3d(&mut self) {

        let vertices: Vec<[f64; 3]> = self.nodes.iter().map(|node| {
            node.arr()
        }).collect();
        self.tetrahedrons.insert_vertices(&vertices, true).unwrap_or_default();
    }

    pub fn add_triangle(&mut self, vertex1: usize, vertex2: usize, vertex3: usize) {

        let p1 = self.tetrahedrons.get_vertices()[vertex1];
        let p2 = self.tetrahedrons.get_vertices()[vertex2];
        let p3 = self.tetrahedrons.get_vertices()[vertex3];

        let n1 = Point3D::new((p1[0] as f32, p1[1] as f32, p1[2] as f32));
        let n2 = Point3D::new((p2[0] as f32, p2[1] as f32, p2[2] as f32));
        let n3 = Point3D::new((p3[0] as f32, p3[1] as f32, p3[2] as f32));

        let l12 = n1.distance(&n2);
        let l23 = n2.distance(&n3);
        let l31 = n3.distance(&n1);

        let edge12 = Edge(min(vertex1, vertex2), max(vertex1, vertex2));
        let edge23 = Edge(min(vertex2, vertex3), max(vertex2, vertex3));
        let edge31 = Edge(min(vertex3, vertex1), max(vertex3, vertex1));

        self.geometry_data.edge_lengths.insert(edge12, l12);
        self.geometry_data.edge_lengths.insert(edge23, l23);
        self.geometry_data.edge_lengths.insert(edge31, l31);

        self.geometry_data.vertex_connections.entry(vertex1).or_insert_with(HashSet::new).insert(vertex2);
        self.geometry_data.vertex_connections.entry(vertex2).or_insert_with(HashSet::new).insert(vertex1);
        
        self.geometry_data.vertex_connections.entry(vertex2).or_insert_with(HashSet::new).insert(vertex3);
        self.geometry_data.vertex_connections.entry(vertex3).or_insert_with(HashSet::new).insert(vertex2);
        
        self.geometry_data.vertex_connections.entry(vertex3).or_insert_with(HashSet::new).insert(vertex1);
        self.geometry_data.vertex_connections.entry(vertex1).or_insert_with(HashSet::new).insert(vertex3);

    }

    pub fn add_tetrahedron(&mut self, vertex1: usize, vertex2: usize, vertex3: usize, vertex4: usize) {

        let p1 = self.tetrahedrons.get_vertices()[vertex1];
        let p2 = self.tetrahedrons.get_vertices()[vertex2];
        let p3 = self.tetrahedrons.get_vertices()[vertex3];
        let p4 = self.tetrahedrons.get_vertices()[vertex4];

        let n1 = Point3D::new((p1[0] as f32, p1[1] as f32, p1[2] as f32));
        let n2 = Point3D::new((p2[0] as f32, p2[1] as f32, p2[2] as f32));
        let n3 = Point3D::new((p3[0] as f32, p3[1] as f32, p3[2] as f32));
        let n4 = Point3D::new((p4[0] as f32, p4[1] as f32, p4[2] as f32));

        let l12 = &n1.distance(&n2);
        let l23 = &n2.distance(&n3);
        let l31 = &n3.distance(&n1);
        let l41 = &n4.distance(&n1);
        let l42 = &n4.distance(&n2);
        let l43 = &n4.distance(&n3);

        let edge12 = Edge(min(vertex1, vertex2), max(vertex1, vertex2));
        let edge23 = Edge(min(vertex2, vertex3), max(vertex2, vertex3));
        let edge31 = Edge(min(vertex3, vertex1), max(vertex3, vertex1));
        let edge41 = Edge(min(vertex4, vertex1), max(vertex4, vertex1));
        let edge42 = Edge(min(vertex4, vertex2), max(vertex4, vertex2));
        let edge43 = Edge(min(vertex4, vertex3), max(vertex4, vertex3));

        self.geometry_data.edge_lengths.insert(edge12, *l12);
        self.geometry_data.edge_lengths.insert(edge23, *l23);
        self.geometry_data.edge_lengths.insert(edge31, *l31);
        self.geometry_data.edge_lengths.insert(edge41, *l41);
        self.geometry_data.edge_lengths.insert(edge42, *l42);
        self.geometry_data.edge_lengths.insert(edge43, *l43);

        self.geometry_data.vertex_connections.entry(vertex1).or_insert_with(HashSet::new).insert(vertex2);
        self.geometry_data.vertex_connections.entry(vertex2).or_insert_with(HashSet::new).insert(vertex1);
        
        self.geometry_data.vertex_connections.entry(vertex2).or_insert_with(HashSet::new).insert(vertex3);
        self.geometry_data.vertex_connections.entry(vertex3).or_insert_with(HashSet::new).insert(vertex2);
        
        self.geometry_data.vertex_connections.entry(vertex3).or_insert_with(HashSet::new).insert(vertex1);
        self.geometry_data.vertex_connections.entry(vertex1).or_insert_with(HashSet::new).insert(vertex3);
        
        self.geometry_data.vertex_connections.entry(vertex4).or_insert_with(HashSet::new).insert(vertex1);
        self.geometry_data.vertex_connections.entry(vertex1).or_insert_with(HashSet::new).insert(vertex4);
        
        self.geometry_data.vertex_connections.entry(vertex4).or_insert_with(HashSet::new).insert(vertex2);
        self.geometry_data.vertex_connections.entry(vertex2).or_insert_with(HashSet::new).insert(vertex4);
        
        self.geometry_data.vertex_connections.entry(vertex4).or_insert_with(HashSet::new).insert(vertex3);
        self.geometry_data.vertex_connections.entry(vertex3).or_insert_with(HashSet::new).insert(vertex4);

    }

    pub fn preprocess_3d(&mut self) {
        let structure = self.tetrahedrons.get_simplicial();
        let num_tetras = structure.get_nb_tetrahedra();
    
        for tetra_idx in 0..num_tetras {
            let tetrahedron = match self.tetrahedrons.get_simplicial().get_tetrahedron(tetra_idx) {
                Ok(tetra) => tetra.nodes(),
                Err(_) => {
                    println!("Error getting tetrahedron at index {}", tetra_idx);
                    continue; // Skip this iteration and proceed with the next one
                }
            };
    
            let [node1, node2, node3, node4] = tetrahedron;
            match (node1, node2, node3, node4) {
                (Node::Infinity, Node::Value(ind_v2), Node::Value(ind_v3), Node::Value(ind_v4)) => {
                    self.add_triangle(ind_v2, ind_v3, ind_v4);
                },
                (Node::Value(ind_v1), Node::Infinity, Node::Value(ind_v3), Node::Value(ind_v4)) => {
                    self.add_triangle(ind_v3, ind_v4, ind_v1);
                },
                (Node::Value(ind_v1), Node::Value(ind_v2), Node::Infinity, Node::Value(ind_v4)) => {
                    self.add_triangle(ind_v4, ind_v1, ind_v2);
                },
                (Node::Value(ind_v1), Node::Value(ind_v2), Node::Value(ind_v3), Node::Infinity) => {
                    self.add_triangle(ind_v1, ind_v2, ind_v3);
                },
                (Node::Value(ind_v1), Node::Value(ind_v2), Node::Value(ind_v3), Node::Value(ind_v4)) => {
                    self.add_tetrahedron(ind_v1, ind_v2, ind_v3, ind_v4);
                },
                (_, _, _, _) => {
                    println!("Encountered an unexpected pattern of nodes");
                },
            };
        }
    }
    

}
use xenobalanus::Xenobalanus;

#[test]
fn threed() {
    let mut geodesic = Xenobalanus::new();
    geodesic.random_points_3d((0.0, 0.0), 1.0, 1000);
    geodesic.delaunay_3d();
    geodesic.preprocess_3d();
    let attractors = geodesic.dtscan(5, 0.085);

    println!("{:#?}", attractors);
}
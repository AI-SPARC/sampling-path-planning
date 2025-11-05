mod structs3d;
mod readmap3d;
mod rrt3d;
mod readmap; 
mod structs; 

use readmap3d::OccupancyMap3D;
use rrt3d::RRTPlanner3D;
use structs3d::Point3;

fn main() {
    let depth = 20;
    let dz = 1.0;

    let map3d = OccupancyMap3D::replicate_from("data/map.jpg", depth, dz);

    let start = Point3 { x: 50.0,  y: 50.0,  z: 0.0};
    let goal  = Point3 { x: 700.0, y: 500.0, z: (depth as f64 - 1.0) * dz };

    let step_size   = 15.0;
    let goal_radius = 10.0;

    let mut planner = RRTPlanner3D::new(start, goal, map3d, step_size, goal_radius).with_goal_bias(0.05).with_max_iter(20_000).with_collision_steps(20);

    match planner.plan() {
        Some(path) => {
            println!("Caminho 3D encontrado! Tamanho: {}", path.len());
            let _ = planner.save_nodes_to_csv("data/rrt_nodes_3d.csv");
            let _ = planner.save_path_to_csv("data/rrt_path_3d.csv");
        }
        None => {
            println!("Não foi possível encontrar caminho em 3D.");
        }
    }
}

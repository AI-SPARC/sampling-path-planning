mod structs;
mod readmap;
mod rrt;

use crate::structs::*;
use crate::readmap::*;
use crate::rrt::*;

fn main() {

    let map = OccupancyMap::new("data/map.jpg"); // Lê o mapa

    let start_point = Point { x: 50.0, y: 50.0 }; // Exemplo
    let goal_point = Point { x: 700.0, y: 500.0 }; // Exemplo
    let step_size = 15.0; // 15 pixels por passo
    let goal_radius = 10.0; // Considera o objetivo alcançado se estiver a 10 pixels de distância
    let max_iterations = 10000;
    let num_collision_check_steps = 10; // Pontos para verificação entre q_nearst e q_new

    let mut planner = RRTPlanner::new( start_point, goal_point, map, step_size, goal_radius, max_iterations, num_collision_check_steps);

    println!("Iniciando planejamento RRT...");
    if let Some(path) = planner.RRT() {
        println!("\nCaminho encontrado com {} pontos:", path.len());
        for point in path {
            println!("  ({:.2}, {:.2})", point.x, point.y); 
        }
        if let Err(e) = planner.save_all_nodes_to_csv("data/rrt_nodes.csv") {
            eprintln!("Erro ao salvar nós: {}", e);
        } else {
            println!("Nós da árvore salvos em rrt_nodes.csv");
        }

        if let Err(e) = planner.save_final_path_to_csv("data/rrt_path.csv") {
            eprintln!("Erro ao salvar caminho: {}", e);
        } else {
            println!("Caminho final salvo em rrt_path.csv");
        }

    } else {
        println!("Não foi possível encontrar um caminho.");
    }
}
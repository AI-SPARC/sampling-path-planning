use rand::Rng;
use std::f64;

use crate::structs::*;
use crate::readmap::*;

pub struct RRTPlanner {
    pub start: Point,
    pub goal: Point,
    pub map: OccupancyMap,
    pub step_size: f64,
    pub goal_radius: f64,
    pub max_iter: u32,
    pub num_collision_check_steps: i32,
    pub tree: RRTTree,
}

impl RRTPlanner {
    pub fn new( start: Point, goal: Point, map: OccupancyMap, step_size: f64, goal_radius: f64, max_iter: u32, num_collision_check_steps: i32,) -> Self {
        RRTPlanner {
            start,
            goal,
            map,
            step_size,
            goal_radius,
            max_iter,
            num_collision_check_steps,
            tree: RRTTree::new(start),
        }
    }

    // Gera um ponto aleatório dentro dos limites
    pub fn random_point(&self) -> Point {
        let mut rng = rand::thread_rng();
        let (x_min_px, y_min_px, x_max_px, y_max_px) = self.map.pixel_bounds;
        Point {
            // Tranforma em f64 para conseguir navegar por todo mapa
            // Número gerado por distribuição uniforme
            x: rng.gen_range(x_min_px as f64..=x_max_px as f64),
            y: rng.gen_range(y_min_px as f64..=y_max_px as f64),
        }
    }

    // Cria o ponto a partir do aleatório e o step size
    pub fn create_step(&self, nearest_coord: &Point, q_rand: &Point) -> Point {
        let dist = nearest_coord.dist(q_rand);
        if dist < self.step_size {
            return *q_rand;
        }

        let ratio = self.step_size / dist;
        Point {
            x: nearest_coord.x + (q_rand.x - nearest_coord.x) * ratio,
            y: nearest_coord.y + (q_rand.y - nearest_coord.y) * ratio,
        }
    }

    pub fn RRT(&mut self) -> Option<Vec<Point>> {
        for _ in 0..self.max_iter {
            let q_rand = self.random_point();
            let nearest_node_idx = self.tree.nearest_node_idx(&q_rand);
            let nearest_node_coord = self.tree.get_node(nearest_node_idx).unwrap().coord;
            
            let q_new = self.create_step(&nearest_node_coord, &q_rand);

            if !self.map.is_path_colliding(&nearest_node_coord, &q_new, self.num_collision_check_steps) {
                let new_node_idx = self.tree.add_node(q_new, nearest_node_idx);

                if q_new.dist(&self.goal) <= self.goal_radius {
                    let mut path = self.tree.reconstruct_path(new_node_idx);
                    if path.last().map_or(false, |p| p != &self.goal) {
                        path.push(self.goal);
                    }
                    return Some(path);
                }
            }
        }
        println!("Falha ao encontrar um caminho após {} iterações.", self.max_iter);
        None
    }
}
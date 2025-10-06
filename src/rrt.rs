use rand::Rng;
use std::f64;

use crate::structs::*;
use crate::readmap::*;

pub struct RRTPlanner {
    start: Point,
    goal: Point,
    map: OccupancyMap,
    step_size: f64,
    goal_radius: f64,
    max_iter: u32,
    num_collision_check_steps: i32,
    
    tree: RRTTree,
    path_found: Option<Vec<Point>>,
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
            path_found: None,
        }
    }

     // Salva todos os nós da árvore em um arquivo CSV
    pub fn save_all_nodes_to_csv(&self, filename: &str) -> Result<(), std::io::Error> {
        use std::io::Write; 
        let mut file = std::fs::File::create(filename)?;
        writeln!(file, "x,y,parent_x,parent_y")?;

        for (i, node) in self.tree.nodes().iter().enumerate() {
            let parent_coords = if let Some(parent_idx) = node.parent_idx {
                let parent_node = self.tree.get_node(parent_idx).unwrap();
                format!("{:.2},{:.2}", parent_node.coord.x, parent_node.coord.y)
            } else {
                "NaN,NaN".to_string() 
            };
            writeln!(file, "{:.2},{:.2},{}", node.coord.x, node.coord.y, parent_coords)?;
        }
        Ok(())
    }

    // Salva o caminho final em um arquivo CSV
    pub fn save_final_path_to_csv(&self, filename: &str) -> Result<(), std::io::Error> {
        use std::io::Write;
        if let Some(path) = &self.path_found {
            let mut file = std::fs::File::create(filename)?;
            writeln!(file, "x,y")?;
            for point in path {
                writeln!(file, "{:.2},{:.2}", point.x, point.y)?;
            }
            Ok(())
        } else {
            Err(std::io::Error::new(std::io::ErrorKind::NotFound, "Nenhum caminho final para salvar."))
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

    pub fn RRT(&mut self) -> Option<&Vec<Point>> {
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
                    self.path_found = Some(path);
                    return self.path_found.as_ref();
                }
            }
        }
        println!("Falha ao encontrar um caminho após {} iterações.", self.max_iter);
        None
    }
}
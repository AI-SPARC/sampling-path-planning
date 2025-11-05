use rand::Rng;

use crate::readmap3d::OccupancyMap3D;
use crate::structs3d::{Point3, RRTTree3D};

pub struct RRTPlanner3D {
    start: Point3,
    goal: Point3,
    map: OccupancyMap3D,
    step_size: f64,
    goal_radius: f64,
    goal_bias: f64,
    max_iter: u32,
    collision_steps: i32,

    tree: RRTTree3D,
    path_found: Option<Vec<Point3>>,
}

impl RRTPlanner3D {
    pub fn new(start: Point3, goal: Point3, map: OccupancyMap3D, step_size: f64, goal_radius: f64) -> Self {
        Self {
            start,
            goal,
            map,
            step_size,
            goal_radius,
            goal_bias: 0.05,
            max_iter: 20_000, 
            collision_steps: 20,
            tree: RRTTree3D::new(start),
            path_found: None,
        }
    }

    pub fn with_goal_bias(mut self, bias: f64) -> Self {
        self.goal_bias = bias.clamp(0.0, 1.0);
        self
    }

    pub fn with_max_iter(mut self, iters: u32) -> Self {
        self.max_iter = iters;
        self
    }

    pub fn with_collision_steps(mut self, steps: i32) -> Self {
        self.collision_steps = steps.max(1);
        self
    }

    #[inline]
    fn random_point(&self) -> Point3 {
        let mut rng = rand::thread_rng();
        if rand::Rng::r#gen::<f64>(&mut rng) < self.goal_bias {
            return self.goal;
        }
        let x = rng.gen_range(0.0..self.map.width as f64);
        let y = rng.gen_range(0.0..self.map.height as f64);
        let z = rng.gen_range(0.0..(self.map.depth as f64) * self.map.dz);
        Point3 { x, y, z }
    }

    #[inline]
    fn steer(&self, from: &Point3, to: &Point3) -> Point3 {
        let dx = to.x - from.x;
        let dy = to.y - from.y;
        let dz = to.z - from.z;
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        if d <= self.step_size {
            *to
        } else {
            let s = self.step_size / d;
            Point3 {
                x: from.x + dx * s,
                y: from.y + dy * s,
                z: from.z + dz * s,
            }
        }
    }

    pub fn plan(&mut self) -> Option<&Vec<Point3>> {
        for _ in 0..self.max_iter {
            let q_rand = self.random_point();
            let nearest_idx = self.tree.nearest(&q_rand);
            let q_near = self.tree.nodes[nearest_idx].coord;
            let q_new = self.steer(&q_near, &q_rand);

            if !self.map.is_colliding(&q_near, &q_new, self.collision_steps){
                let new_idx = self.tree.add_node(q_new, nearest_idx);

                if q_new.dist(&self.goal) <= self.goal_radius {
                    let mut path = self.tree.reconstruct_path(new_idx);
                    if path.last().map_or(true, |p| p != &self.goal) {
                        path.push(self.goal);
                    }
                    self.path_found = Some(path);
                    return self.path_found.as_ref();
                }
            }
        }
        None
    }

    pub fn path(&self) -> Option<&Vec<Point3>> {
        self.path_found.as_ref()
    }

    pub fn save_nodes_to_csv(&self, out: &str) -> std::io::Result<()> {
        let mut w = csv::Writer::from_path(out)?;
        w.write_record(&["x", "y", "z", "parent_idx"])?;
        for (i, n) in self.tree.nodes.iter().enumerate() {
            let parent = n
                .parent_idx
                .map(|p| p.to_string())
                .unwrap_or_else(|| "-1".to_string());
            w.write_record(&[
                n.coord.x.to_string(),
                n.coord.y.to_string(),
                n.coord.z.to_string(),
                parent,
            ])?;
        }
        w.flush()?;
        Ok(())
    }

    pub fn save_path_to_csv(&self, out: &str) -> std::io::Result<()> {
        if let Some(path) = &self.path_found {
            let mut w = csv::Writer::from_path(out)?;
            w.write_record(&["x", "y", "z"])?;
            for p in path {
                w.write_record(&[p.x.to_string(), p.y.to_string(), p.z.to_string()])?;
            }
            w.flush()?;
        }
        Ok(())
    }
}

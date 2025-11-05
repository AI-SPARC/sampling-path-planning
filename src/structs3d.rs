#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    #[inline]
    pub fn dist(&self, other: &Point3) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2)).sqrt()
    }
}

#[derive(Debug, Clone)]
pub struct RRTNode3D {
    pub coord: Point3,
    pub parent_idx: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct RRTTree3D {
    pub nodes: Vec<RRTNode3D>,
}

impl RRTTree3D {
    pub fn new(start: Point3) -> Self {
        Self {
            nodes: vec![RRTNode3D {
                coord: start,
                parent_idx: None,
            }],
        }
    }

    #[inline]
    pub fn add_node(&mut self, coord: Point3, parent_idx: usize) -> usize {
        self.nodes.push(RRTNode3D {coord, parent_idx: Some(parent_idx),});
        self.nodes.len() - 1
    }

    #[inline]
    pub fn nearest(&self, p: &Point3) -> usize {
        let mut best = 0usize;
        let mut best_d = f64::INFINITY;
        for (i, n) in self.nodes.iter().enumerate() {
            let d = n.coord.dist(p);
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        best
    }

    pub fn reconstruct_path(&self, end_node_idx: usize) -> Vec<Point3> {
        let mut path = Vec::new();
        let mut cur = Some(end_node_idx);
        while let Some(i) = cur {
            path.push(self.nodes[i].coord);
            cur = self.nodes[i].parent_idx;
        }
        path.reverse();
        path
    }
}

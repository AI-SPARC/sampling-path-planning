pub use rand::Rng;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn dist(&self, other: &Point) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

#[derive(Debug)]
pub struct RRTNode {
    pub coord: Point,
    pub parent_idx: Option<usize>, // Índice do nó pai no vetor da árvore
}

pub struct RRTTree {
    pub nodes: Vec<RRTNode>,
}

impl RRTTree {
    pub fn new(start_point: Point) -> Self {
        RRTTree {
            nodes: vec![RRTNode { coord: start_point, parent_idx: None }],
        }
    }

    pub fn add_node(&mut self, point: Point, parent_idx: usize) -> usize {
        let new_node = RRTNode { coord: point, parent_idx: Some(parent_idx) };
        self.nodes.push(new_node);
        self.nodes.len() - 1 // Retorna o índice do novo nó
    }

    pub fn get_node(&self, idx: usize) -> Option<&RRTNode> {
        self.nodes.get(idx)
    }

    pub fn nodes(&self) -> &Vec<RRTNode> {
        &self.nodes
    }

    // Encontra o índice do nó mais próximo na árvore a um dado ponto
    pub fn nearest_node_idx(&self, point: &Point) -> usize {
        let mut nearest_idx = 0;
        let mut min_dist = self.nodes[0].coord.dist(point);

        for (i, node) in self.nodes.iter().enumerate().skip(1) {
            let dist = node.coord.dist(point);
            if dist < min_dist {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        nearest_idx
    }

    // Reconstrói o caminho do nó final até o nó inicial
    pub fn reconstruct_path(&self, end_node_idx: usize) -> Vec<Point> {
        let mut path = Vec::new();
        let mut current_idx = Some(end_node_idx);
        while let Some(idx) = current_idx {
            path.push(self.nodes[idx].coord);
            current_idx = self.nodes[idx].parent_idx;
        }
        path.reverse();
        path
    }
}
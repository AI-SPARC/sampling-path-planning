use image::{ImageBuffer, Rgb};
use crate::readmap::is_black;
use crate::structs3d::Point3;

pub struct OccupancyMap3D {
    // Vetor de camadas: [z][y][x]
    pub layers: Vec<ImageBuffer<Rgb<u8>, Vec<u8>>>,
    pub width: u32,
    pub height: u32,
    pub depth: usize, // número de slices
    pub dz: f64,      // espessura
}

impl OccupancyMap3D {
    
    pub fn replicate_from(file_path: &str, depth: usize, dz: f64) -> Self {
        let img = image::open(file_path).expect("Não foi possível abrir a imagem do mapa (3D).").to_rgb8();
        let (width, height) = img.dimensions();
        let mut layers = Vec::with_capacity(depth);
        for _ in 0..depth {
            layers.push(img.clone());
        }

        Self {
            layers,
            width,
            height,
            depth,
            dz,
        }
    }

    /*
    #[allow(dead_code)]
    pub fn from_files(paths: &[&str], dz: f64) -> Self {
        assert!(!paths.is_empty(), "É necessário pelo menos um arquivo para as camadas");
        let mut layers: Vec<ImageBuffer<Rgb<u8>, Vec<u8>>> = Vec::with_capacity(paths.len());
        for p in paths {
            layers.push(
                image::open(p).unwrap_or_else(|_| panic!("Falha ao abrir a imagem da camada: {p}")).to_rgb8(),
            );
        }
        let (width, height) = layers[0].dimensions();
        for (i, l) in layers.iter().enumerate() {
            let (w, h) = l.dimensions();
            assert_eq!((w, h),(width, height),"Todas as camadas devem ter o mesmo tamanho. Camada {i} difere.");
        }
        Self {
            layers,
            width,
            height,
            depth: layers.len(),
            dz,
        }
    }
     */

    #[inline]
    fn clamp_xy(&self, x: i32, y: i32) -> Option<(u32, u32)> {
        if x >= 0 && y >= 0 && (x as u32) < self.width && (y as u32) < self.height {
            Some((x as u32, y as u32))
        } else {
            None
        }
    }

    #[inline]
    fn clamp_z(&self, z_idx: i32) -> Option<usize> {
        if z_idx >= 0 && (z_idx as usize) < self.depth {
            Some(z_idx as usize)
        } else {
            None
        }
    }

    // Checa se (x,y,z) cai em obstáculo (pixel preto) — z contínuo é mapeado para slice.
    #[inline]
    pub fn is_obstructed_xyz(&self, x: f64, y: f64, z: f64) -> bool {
        let xi = x.round() as i32;
        let yi = y.round() as i32;
        let zi = (z / self.dz).round() as i32;

        if let (Some((xu, yu)), Some(zu)) = (self.clamp_xy(xi, yi), self.clamp_z(zi)) {
            let px = self.layers[zu].get_pixel(xu, yu);
            is_black(px)
        } else {
            true 
        }
    }

    pub fn is_colliding(&self, a: &Point3, b: &Point3, num_steps: i32) -> bool {
        let steps = num_steps.max(1) as f64;
        let dx = (b.x - a.x) / steps;
        let dy = (b.y - a.y) / steps;
        let dz = (b.z - a.z) / steps;

        for i in 0..=steps as i32 {
            let p = Point3 {
                x: a.x + dx * i as f64,
                y: a.y + dy * i as f64,
                z: a.z + dz * i as f64,
            };
            if self.is_obstructed_xyz(p.x, p.y, p.z) {
                return true;
            }
        }
        false
    }
}

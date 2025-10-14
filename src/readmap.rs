use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;

use crate::structs::*;
pub use image::{ImageBuffer, Rgb};


pub fn is_black(pixel: &Rgb<u8>) -> bool {
    let threshold = 50;
    pixel[0] < threshold && pixel[1] < threshold && pixel[2] < threshold
}

pub fn find_bounds(image: &ImageBuffer<Rgb<u8>, Vec<u8>>) -> (u32, u32, u32, u32) {
    
    let (width, height) = image.dimensions();
    let mut y_min = 0;
    let mut y_max = height - 1;
    let mut x_min = 0;
    let mut x_max = width - 1;

    for y in 0..height {
        let mut found_non_black = false;
        for x in 0..width {
            if !is_black(image.get_pixel(x, y)) {
                y_min = y;
                found_non_black = true;
                break;
            }
        }
        if found_non_black { break; }
    }

    for y in (0..height).rev() {
        let mut found_non_black = false;
        for x in 0..width {
            if !is_black(image.get_pixel(x, y)) {
                y_max = y;
                found_non_black = true;
                break;
            }
        }
        if found_non_black { break; }
    }

    for x in 0..width {
        let mut found_non_black = false;
        for y in y_min..=y_max {
            if !is_black(image.get_pixel(x, y)) {
                x_min = x;
                found_non_black = true;
                break;
            }
        }
        if found_non_black { break; }
    }

    for x in (0..width).rev() {
        let mut found_non_black = false;
        for y in y_min..=y_max {
            if !is_black(image.get_pixel(x, y)) {
                x_max = x;
                found_non_black = true;
                break;
            }
        }
        if found_non_black { break; }
    }
    
    (x_min, y_min, x_max, y_max)
}

pub struct OccupancyMap {
    pub image: ImageBuffer<Rgb<u8>, Vec<u8>>,
    pub pixel_bounds: (u32, u32, u32, u32), // (x_min, y_min, x_max, y_max)
}

impl OccupancyMap {
    pub fn new(file_path: &str) -> Self {
        let img = image::open(file_path).expect("Não foi possível abrir a imagem do mapa.");
        let rgb_img = img.to_rgb8();

        let pixel_bounds = find_bounds(&rgb_img);
        println!("Limites de navegação detectados (pixels): {:?}", pixel_bounds);

        OccupancyMap {
            image: rgb_img,
            pixel_bounds,
        }
    }

    // Verifica se um ponto específico está em um obstáculo (pixel preto)
    pub fn is_obstructed(&self, point: &Point) -> bool {
        let (x_min_px, y_min_px, x_max_px, y_max_px) = self.pixel_bounds;
        
        // Arredondamos para o pixel mais próximo para obter a coordenada inteira.
        let px = point.x.round() as u32;
        let py = point.y.round() as u32;

        let (img_width, img_height) = self.image.dimensions();
        if px >= img_width || py >= img_height {
            // Isso só deve acontecer se o ponto estiver exatamente na borda externa do pixel_bounds
            return true; 
        }

        let pixel = self.image.get_pixel(px, py);
        is_black(pixel)
    }

    // Verifica o caminho
    pub fn is_path_colliding(&self, start: &Point, end: &Point, num_steps: i32) -> bool {
        let dx = end.x - start.x;
        let dy = end.y - start.y;

        for i in 0..=num_steps {
            let t = i as f64 / num_steps as f64;
            let check_point = Point {
                x: start.x + t * dx,
                y: start.y + t * dy,
            };

            if self.is_obstructed(&check_point) {
                return true; // Colisão
            }
        }
        false
    }
}

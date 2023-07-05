use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Debug, PartialEq, Eq)]
struct Vertex {
    row: usize,
    col: usize,
    distance: i32,
}

impl Ord for Vertex {
    fn cmp(&self, other: &Self) -> Ordering {
        other.distance.cmp(&self.distance)
    }
}

impl PartialOrd for Vertex {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug)]
struct Map {
    rows: usize,
    cols: usize,
    obstacles: Vec<(usize, usize)>,
}

impl Map {
    fn new(rows: usize, cols: usize) -> Self {
        Map {
            rows,
            cols,
            obstacles: Vec::new(),
        }
    }

    fn add_obstacle(&mut self, row: usize, col: usize) {
        if row < self.rows && col < self.cols {
            self.obstacles.push((row, col));
        }
    }

    fn generate_obstacles(&mut self, num_obstacles: usize) {
        use rand::Rng;

        let mut rng = rand::thread_rng();

        for _ in 0..num_obstacles {
            let row = rng.gen_range(0..self.rows);
            let col = rng.gen_range(0..self.cols);
            self.add_obstacle(row, col);
        }
    }

    fn dijkstra(
        &self,
        source: (usize, usize),
        destination: (usize, usize),
    ) -> Option<Vec<(usize, usize)>> {
        let mut distances: Vec<_> = (0..self.rows * self.cols).map(|_| i32::MAX).collect();
        let mut previous: Vec<_> = (0..self.rows * self.cols).map(|_| None).collect();
        let mut priority_queue = BinaryHeap::new();

        let source_index = self.coords_to_index(source.0, source.1);
        let destination_index = self.coords_to_index(destination.0, destination.1);

        distances[source_index] = 0;
        priority_queue.push(Vertex {
            row: source.0,
            col: source.1,
            distance: 0,
        });

        while let Some(Vertex { row, col, distance }) = priority_queue.pop() {
            let current_index = self.coords_to_index(row, col);

            if current_index == destination_index {
                // Reconstruct the path
                let mut path = vec![(row, col)];
                let mut current = current_index;
                while let Some(prev_index) = previous[current] {
                    let (prev_row, prev_col) = self.index_to_coords(prev_index);
                    path.push((prev_row, prev_col));
                    current = prev_index;
                }
                path.reverse();
                return Some(path);
            }

            if distance > distances[current_index] {
                continue;
            }

            let neighbors = self.get_neighbors(row, col);
            for (neighbor_row, neighbor_col) in neighbors {
                let neighbor_index = self.coords_to_index(neighbor_row, neighbor_col);
                let new_distance = distance + self.cost(row, col, neighbor_row, neighbor_col);
                if new_distance < distances[neighbor_index] {
                    distances[neighbor_index] = new_distance;
                    previous[neighbor_index] = Some(current_index);
                    priority_queue.push(Vertex {
                        row: neighbor_row,
                        col: neighbor_col,
                        distance: new_distance,
                    });
                }
            }
        }

        None
    }

    fn cost(&self, row1: usize, col1: usize, row2: usize, col2: usize) -> i32 {
        let dx = (row1 as i32 - row2 as i32).abs();
        let dy = (col1 as i32 - col2 as i32).abs();
        if dx == 0 || dy == 0 {
            1
        } else {
            2
        }
    }

    fn coords_to_index(&self, row: usize, col: usize) -> usize {
        row * self.cols + col
    }

    fn index_to_coords(&self, index: usize) -> (usize, usize) {
        (index / self.cols, index % self.cols)
    }

    fn get_neighbors(&self, row: usize, col: usize) -> Vec<(usize, usize)> {
        let mut neighbors = Vec::new();

        let directions = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ];

        for (dx, dy) in &directions {
            let neighbor_row = row as i32 + dx;
            let neighbor_col = col as i32 + dy;

            if neighbor_row >= 0
                && neighbor_row < self.rows as i32
                && neighbor_col >= 0
                && neighbor_col < self.cols as i32
            {
                let neighbor_row = neighbor_row as usize;
                let neighbor_col = neighbor_col as usize;
                if !self.obstacles.contains(&(neighbor_row, neighbor_col)) {
                    neighbors.push((neighbor_row, neighbor_col));
                }
            }
        }

        neighbors
    }

    fn display_with_path(
        &self,
        path: &[usize],
        source: (usize, usize),
        destination: (usize, usize),
    ) {
        let mut map = vec![vec![' '; self.cols]; self.rows];

        for (row, col) in &self.obstacles {
            map[*row][*col] = '#';
        }

        for index in path {
            let (row, col) = self.index_to_coords(*index);
            map[row][col] = '*';
        }

        map[source.0][source.1] = 'S';
        map[destination.0][destination.1] = 'T';

        for row in map {
            for cell in row {
                print!("{} ", cell);
            }
            println!();
        }
    }
}

fn main() {
    let rows = 10;
    let cols = 10;

    let mut map = Map::new(rows, cols);
    map.generate_obstacles(15);

    let source = (1, 7);
    let destination = (8, 7);

    if let Some(path) = map.dijkstra(source, destination) {
        let indices: Vec<usize> = path
            .iter()
            .map(|&(row, col)| map.coords_to_index(row, col))
            .collect();

        println!("Shortest path from {:?} to {:?}:", destination, source);
        map.display_with_path(&indices, source, destination);
    } else {
        println!("No path found from {:?} to {:?}.", source, destination);
    }
}

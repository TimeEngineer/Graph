use core::fmt::{Debug, Formatter};
use core::ops::Add;

pub trait AdjencyMatrix<T: Copy + PartialOrd + Add<Output = T> + Debug> {
    fn inner(&self) -> &[T];
    fn nb_vertex(&self) -> usize;
    fn infinite(&self) -> T;
    #[inline]
    fn neighbors(&self, i: usize) -> &[T] {
        let nb_vertex = self.nb_vertex();
        &self.inner()[i * nb_vertex..(i + 1) * nb_vertex]
    }
    #[inline]
    fn edge(&self, i: usize, j: usize) -> T {
        let nb_vertex = self.nb_vertex();
        self.inner()[i * nb_vertex + j]
    }
    #[inline]
    fn dijkstra_init(&self, src: usize, visited: &mut [bool], distance: &mut [T]) {
        for x in visited.iter_mut() {
            *x = false;
        }
        distance.copy_from_slice(self.neighbors(src));
        visited[src] = true;
    }
    #[inline]
    fn dijkstra_with_path_init(
        &self,
        src: usize,
        path: &mut [usize],
        visited: &mut [bool],
        distance: &mut [T],
    ) {
        for x in visited.iter_mut() {
            *x = false;
        }
        for x in path.iter_mut() {
            *x = src;
        }
        distance.copy_from_slice(self.neighbors(src));
        visited[src] = true;
    }
    #[inline]
    fn visits(&self, visited: &mut [bool], distance: &mut [T]) -> (usize, T) {
        let infinite = self.infinite();

        let mut min = (0, distance[0]);
        for d in distance.iter().copied().enumerate().skip(1) {
            if min.1 == infinite || (d.1 != infinite && d.1 < min.1) {
                min = d;
            }
        }
        visited[min.0] = true;
        distance[min.0] = infinite;
        min
    }
    #[inline]
    fn update_distance(&self, cur: (usize, T), visited: &[bool], distance: &mut [T]) {
        let infinite = self.infinite();
        let nb_vertex = self.nb_vertex();
        let visited = &visited[..nb_vertex];
        let distance = &mut distance[..nb_vertex];

        for i in 0..nb_vertex {
            let edge = self.edge(cur.0, i);
            let distance_i = &mut distance[i];

            if !visited[i] {
                let new_dist = cur.1 + edge;
                if edge != infinite && (*distance_i == infinite || *distance_i > new_dist) {
                    *distance_i = new_dist;
                }
            }
        }
    }
    #[inline]
    fn update_distance_with_path(
        &self,
        cur: (usize, T),
        path: &mut [usize],
        visited: &[bool],
        distance: &mut [T],
    ) {
        let infinite = self.infinite();
        let nb_vertex = self.nb_vertex();
        let path = &mut path[..nb_vertex];
        let visited = &visited[..nb_vertex];
        let distance = &mut distance[..nb_vertex];

        for i in 0..nb_vertex {
            let edge = self.edge(cur.0, i);
            let distance_i = &mut distance[i];
            let path_i = &mut path[i];

            if !visited[i] {
                let new_dist = cur.1 + edge;
                if edge != infinite && (*distance_i == infinite || *distance_i > new_dist) {
                    *distance_i = new_dist;
                    *path_i = cur.0;
                }
            }
        }
    }
    #[inline]
    fn rebuild_path(&self, src: usize, dst: usize, path: &[usize]) -> Vec<usize> {
        let nb_vertex = self.nb_vertex();
        let mut p = Vec::with_capacity(nb_vertex);
        let mut last_vertex = dst;

        p.push(last_vertex);
        while last_vertex != src {
            p.push(path[last_vertex]);
            last_vertex = *p.last().unwrap();
        }
        p
    }
    fn dijkstra(&self, src: usize, dst: usize, visited: &mut [bool], distance: &mut [T]) -> T {
        let nb_vertex = self.nb_vertex();
        let visited = &mut visited[..nb_vertex];
        let distance = &mut distance[..nb_vertex];

        self.dijkstra_init(src, visited, distance);
        for _ in 0..(nb_vertex - 1) {
            let cur = self.visits(visited, distance);
            self.update_distance(cur, visited, distance);
            if cur.0 == dst {
                return cur.1;
            }
        }
        self.infinite()
    }
    fn dijkstra_with_path(
        &self,
        src: usize,
        dst: usize,
        path: &mut [usize],
        visited: &mut [bool],
        distance: &mut [T],
    ) -> (T, Vec<usize>) {
        let nb_vertex = self.nb_vertex();
        let path = &mut path[..nb_vertex];
        let visited = &mut visited[..nb_vertex];
        let distance = &mut distance[..nb_vertex];

        self.dijkstra_with_path_init(src, path, visited, distance);
        for _ in 0..(nb_vertex - 1) {
            let cur = self.visits(visited, distance);
            self.update_distance_with_path(cur, path, visited, distance);
            if cur.0 == dst {
                return (cur.1, self.rebuild_path(src, dst, path));
            }
        }
        (self.infinite(), self.rebuild_path(src, dst, path))
    }
    fn dijkstra_from_src(&self, src: usize, visited: &mut [bool], distance: &mut [T]) -> Vec<T> {
        let infinite = self.infinite();
        let nb_vertex = self.nb_vertex();
        let visited = &mut visited[..nb_vertex];
        let distance = &mut distance[..nb_vertex];
        let mut output = vec![infinite; nb_vertex];

        self.dijkstra_init(src, visited, distance);
        for _ in 0..(nb_vertex - 1) {
            let cur = self.visits(visited, distance);
            self.update_distance(cur, visited, distance);
            output[cur.0] = cur.1;
        }
        output
    }
    fn dijkstra_from_src_with_path(
        &self,
        src: usize,
        path: &mut [usize],
        visited: &mut [bool],
        distance: &mut [T],
    ) -> Vec<(T, Vec<usize>)> {
        let nb_vertex = self.nb_vertex();
        let path = &mut path[..nb_vertex];
        let visited = &mut visited[..nb_vertex];
        let distance = &mut distance[..nb_vertex];
        let mut output = Vec::with_capacity(nb_vertex);

        self.dijkstra_with_path_init(src, path, visited, distance);
        for _ in 0..(nb_vertex - 1) {
            let cur = self.visits(visited, distance);
            self.update_distance_with_path(cur, path, visited, distance);
            if cur.1 == self.infinite() { break }
            output.push((cur.1, self.rebuild_path(src, cur.0, path)))
        }
        output
    }
    fn dijkstra_check(
        &self,
        src: usize,
        dst: Option<usize>,
        path: Option<&mut [usize]>,
        visited: &mut [bool],
        distance: &mut [T],
    ) -> Result<(), &str> {
        let nb_vertex = self.nb_vertex();
        let nb_vertex2 = nb_vertex * nb_vertex;
        if self.inner().len() != nb_vertex2 {
            return Err("graph_error: self.len().");
        }
        if src >= nb_vertex {
            return Err("graph_error: src out of scope.");
        }
        if let Some(dst) = dst {
            if dst >= nb_vertex {
                return Err("graph_error: dst out of scope.");
            }
        }
        if let Some(path) = path {
            if path.len() < nb_vertex {
                return Err("graph_error: path.len().");
            }
        }
        if visited.len() < nb_vertex {
            return Err("graph_error: visited.len().");
        }
        if distance.len() < nb_vertex {
            return Err("graph_error: distance.len().");
        }
        Ok(())
    }
}

pub trait AdjencyMatrixNotOriented<T: Copy + PartialOrd + Add<Output = T> + Debug>:
    AdjencyMatrix<T>
{
    fn inner_mut(&mut self) -> &mut [T];
    #[inline]
    fn set_edge(&mut self, i: usize, j: usize, edge: T) {
        let nb_vertex = self.nb_vertex();
        self.inner_mut()[i * nb_vertex + j] = edge;
        self.inner_mut()[j * nb_vertex + i] = edge;
    }
    fn neighbors_mut(&mut self, i: usize) -> &mut [T] {
        let nb_vertex = self.nb_vertex();
        &mut self.inner_mut()[i * nb_vertex..(i + 1) * nb_vertex]
    }
}

impl<'a, T: Debug + Copy + PartialOrd + Add<Output = T> + Debug> Debug for Graph<'a, T> {
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        let nb_vertex = self.nb_vertex();
        for i in 0..nb_vertex {
            writeln!(f, "{:4?}", &self.neighbors(i))?;
        }
        Ok(())
    }
}

pub struct Graph<'a, T: Add<Output = T> + Debug> {
    data: &'a mut [T],
    nb_vertex: usize,
    infinite: T,
}

impl<'a, T: Add<Output = T> + Debug> Graph<'a, T> {
    pub fn from_inner(data: &'a mut [T], nb_vertex: usize, infinite: T) -> Self {
        Self {
            data,
            nb_vertex,
            infinite,
        }
    }
}

impl<'a, T: Copy + PartialOrd + Add<Output = T> + Debug> AdjencyMatrix<T> for Graph<'a, T> {
    fn inner(&self) -> &[T] {
        self.data
    }
    fn nb_vertex(&self) -> usize {
        self.nb_vertex
    }
    fn infinite(&self) -> T {
        self.infinite
    }
}

impl<'a, T: Copy + PartialOrd + Add<Output = T> + Debug> AdjencyMatrixNotOriented<T>
    for Graph<'a, T>
{
    fn inner_mut(&mut self) -> &mut [T] {
        self.data
    }
}

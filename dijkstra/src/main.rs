mod structure;
use crate::structure::Graph;
use crate::structure::{AdjencyMatrix, AdjencyMatrixNotOriented};

type ValueType = u16;
const NB_VERTEX: usize = 10;
const INFINITE: ValueType = 0;
const SRC: usize = 0;
const DST: usize = 5;

fn main() {
    let mut data = vec![INFINITE; NB_VERTEX * NB_VERTEX];
    let mut graph = Graph::from_inner(&mut data, NB_VERTEX, INFINITE);
    graph.set_edge(0, 1, 85);
    graph.set_edge(0, 2, 217);
    graph.set_edge(0, 4, 173);
    graph.set_edge(1, 5, 80);
    graph.set_edge(2, 6, 186);
    graph.set_edge(2, 7, 103);
    graph.set_edge(3, 7, 183);
    graph.set_edge(4, 9, 502);
    graph.set_edge(5, 8, 250);
    graph.set_edge(7, 9, 167);
    graph.set_edge(8, 9, 84);
    println!("\nInput:");
    println!("src: {:?}, dst: {:?}", SRC, DST);
    println!("{:?}", graph);

    let mut path = vec![0; NB_VERTEX];
    let mut visited = vec![false; NB_VERTEX];
    let mut distance = vec![0; NB_VERTEX];

    println!("\nOutput:");
    let cost = graph.dijkstra(SRC, DST, &mut visited, &mut distance);
    println!("cost from {} to {}: {:?}", SRC, DST, cost);
    let cost = graph.dijkstra_with_path(SRC, DST, &mut path, &mut visited, &mut distance);
    println!("cost + path from {} to {}: {:?}", SRC, DST, cost);
    let cost = graph.dijkstra_from_src(SRC, &mut visited, &mut distance);
    println!("cost from {}:", SRC);
    for (i, x) in cost.iter().enumerate() {
        println!("  to {}: {:?}", i, x);
    }
    let cost = graph.dijkstra_from_src_with_path(SRC, &mut path, &mut visited, &mut distance);
    println!("cost + path from {}:", SRC);
    for x in cost {
        println!("  {:?}", x);
    }
}

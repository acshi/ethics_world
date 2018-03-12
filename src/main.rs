#![feature(plugin)]
#![feature(custom_derive)]
#![plugin(rocket_codegen)]

extern crate rocket;
extern crate rocket_contrib;
use rocket_contrib::Template;

use std::hash::{Hash, Hasher};

mod general_search;
use general_search::{PriorityQueue, SearchProblemTrait, SearchNode, create_search_problem, tree_search};

const MAP_WIDTH: usize = 40;
const MAP_HEIGHT: usize = 40;
const MAP_SIZE: usize = MAP_WIDTH * MAP_HEIGHT;

#[derive(Clone)]
struct MapSearchState([u8; MAP_SIZE]);

impl PartialEq for MapSearchState {
    fn eq(&self, other: &MapSearchState) -> bool {
        self.0[..] == other.0[..]
        // let n_sections = MAP_SIZE / 32;
        // for i in 0..n_sections {
        //     let start_i = i * 32;
        //     let end_i = start_i + 32;
        //     if self.0[start_i..end_i] != other.0[start_i..end_i] {
        //         return false;
        //     }
        // }
        // return self.0[n_sections*32..] == other.0[n_sections*32..0];
    }
}

impl Eq for MapSearchState {}

impl Hash for MapSearchState {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.0[..].hash(state);
    }
}

struct MapSearchExpansion(i32);
impl Iterator for MapSearchExpansion {
    type Item = (MapSearchState, i32);
    fn next(&mut self) -> Option<(MapSearchState, i32)> {
        None
    }
}

struct MapSearchTraits;
impl SearchProblemTrait<MapSearchState> for MapSearchTraits {
    fn is_goal(&self, node: &SearchNode<MapSearchState>) -> bool {
        node.state.0[0] == 0
    }

    fn step_cost(&self, node: &SearchNode<MapSearchState>, action: i32) -> f32 {
        action as f32
    }

    fn ordering_cost(&self, node: &SearchNode<MapSearchState>) -> f32 {
        node.path_cost
    }

    fn expand_state(&self, node: &SearchNode<MapSearchState>) -> Box<Iterator<Item = (MapSearchState, i32)>> {
        Box::new(MapSearchExpansion(node.state.0[0] as i32))
    }
}

fn create_map() {
    let mut p = create_search_problem::<MapSearchState, PriorityQueue<SearchNode<MapSearchState>>>
                                        (MapSearchState([0; MAP_SIZE]), &MapSearchTraits{}, true, true);
    {
        let result = tree_search(&mut p);
        if let Some(sol) = result {
            println!("Found solution with cost {:.2} at depth {} after expanding {} nodes", sol.path_cost, sol.depth, sol.get_expansion_count());
            return;
        }
    }
    println!("Falied to find solution after expanding {} nodes", p.get_expansion_count())
}

#[get("/")]
fn index() -> Template {
	Template::render("index", ())
}

fn main() {
    create_map();

    // rocket::ignite()
    //     .mount("/", routes![index])
    //     .attach(Template::fairing())
    //     .launch();
}

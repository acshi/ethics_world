#![feature(plugin)]
#![feature(custom_derive)]
#![plugin(rocket_codegen)]

extern crate rocket;
#[macro_use] extern crate rocket_contrib;
use rocket_contrib::Template;
#[macro_use] extern crate bitflags;
extern crate rand;
extern crate serde;
extern crate serde_json;
#[macro_use] extern crate serde_derive;

use std::hash::{Hash, Hasher};
use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use rocket::{State};
use rocket::response::{NamedFile};
use rocket_contrib::{Json, Value};
use rand::Rng;
use serde::ser::SerializeSeq;

mod general_search;
use general_search::{PriorityQueue, SearchProblemTrait, SearchNode, create_search_problem, tree_search};

// #[derive(Clone)]
// struct MapSearchState([u8; MAP_SIZE]);
//
// impl PartialEq for MapSearchState {
//     fn eq(&self, other: &MapSearchState) -> bool {
//         self.0[..] == other.0[..]
//         // let n_sections = MAP_SIZE / 32;
//         // for i in 0..n_sections {
//         //     let start_i = i * 32;
//         //     let end_i = start_i + 32;
//         //     if self.0[start_i..end_i] != other.0[start_i..end_i] {
//         //         return false;
//         //     }
//         // }
//         // return self.0[n_sections*32..] == other.0[n_sections*32..0];
//     }
// }
//
// impl Eq for MapSearchState {}
//
// impl Hash for MapSearchState {
//     fn hash<H: Hasher>(&self, state: &mut H) {
//         self.0[..].hash(state);
//     }
// }
//
// struct MapSearchExpansion(i32);
// impl Iterator for MapSearchExpansion {
//     type Item = (MapSearchState, i32);
//     fn next(&mut self) -> Option<(MapSearchState, i32)> {
//         None
//     }
// }
//
// struct MapSearchTraits;
// impl SearchProblemTrait<MapSearchState> for MapSearchTraits {
//     fn is_goal(&self, node: &SearchNode<MapSearchState>) -> bool {
//         node.state.0[0] == 0
//     }
//
//     fn step_cost(&self, node: &SearchNode<MapSearchState>, action: i32) -> f32 {
//         action as f32
//     }
//
//     fn ordering_cost(&self, node: &SearchNode<MapSearchState>) -> f32 {
//         node.path_cost
//     }
//
//     fn expand_state(&self, node: &SearchNode<MapSearchState>) -> Box<Iterator<Item = (MapSearchState, i32)>> {
//         Box::new(MapSearchExpansion(node.state.0[0] as i32))
//     }
// }

// fn create_map() {
//     let mut p = create_search_problem::<MapSearchState, PriorityQueue<SearchNode<MapSearchState>>>
//                                         (MapSearchState([0; MAP_SIZE]), &MapSearchTraits{}, true, true);
//     {
//         let result = tree_search(&mut p);
//         if let Some(sol) = result {
//             println!("Found solution with cost {:.2} at depth {} after expanding {} nodes", sol.path_cost, sol.depth, sol.get_expansion_count());
//             return;
//         }
//     }
//     println!("Falied to find solution after expanding {} nodes", p.get_expansion_count())
// }

const MAP_WIDTH: usize = 70;
const MAP_HEIGHT: usize = 70;
const MAP_SIZE: usize = MAP_WIDTH * MAP_HEIGHT;
const BUILDING_N: usize = 12;
const CROSSWALK_N: usize = 6;
const EXTRA_OBSTACLE_N: usize = 6;
const POPULATION_N: usize = 40;
const PEDESTRIAN_N: usize = 10; // those on map at one time
const VEHICLE_N: usize = 10;

const BUILDING_WIDTH: usize = 3; // just a facade
const BUILDING_SPACING: usize = 8;
const CROSSWALK_WIDTH: usize = 2;
const CROSSWALK_SPACING: usize = 6;
const PEDESTRIAN_SIZE: usize = 1;
const VEHICLE_WIDTH: usize = 4;
const VEHICLE_LENGTH: usize = 11;
const SIDEWALK_MIN_WIDTH: usize = 3;
const SIDEWALK_MAX_WIDTH: usize = 6;
const LANE_MIN_WIDTH: usize = 6;
const LANE_MAX_WIDTH: usize = 10;

bitflags! {
    struct Cell: u8 {
        const OUTER_LANE =  0b00000001;
        const INNER_LANE =  0b00000010;
        const OBSTALCE =    0b00000100;
        const CROSSWALK =   0b00001000;
    }
}

impl serde::Serialize for Cell {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where S: serde::Serializer
    {
        serializer.serialize_u8(self.bits)
    }
}

impl fmt::Display for Cell {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.bits)
    }
}

#[derive(Clone, Serialize)]
struct WorldMap {
    grid: Box<[Cell]>,
    buildings: [(usize, usize); BUILDING_N],
    grid_width: usize,
}

fn fill_map_rect(grid: &mut [Cell], left: usize, top: usize, width: usize, height: usize, mask: Cell) {
    // println!("left: {} top: {} width: {} height: {}", left, top, width, height);
    for y in top..(top + height) {
        let mut grid_i = y * MAP_WIDTH + left;
        for _ in 0..width {
            grid[grid_i] |= mask;
            grid_i += 1;
        }
    }
}

fn create_map() -> WorldMap {
    // The world will have a single block of road, making a rectangle.
    let mut grid = [Cell{bits: 0u8}; MAP_SIZE];
    let mut buildings = [(0usize, 0usize); BUILDING_N];

    let mut rng = rand::thread_rng();

    // we make the horizontal (at top and bottom) roads have the same parameters
    // and the vertical ones have their own
    let horiz_road_width = 2 * rng.gen_range(LANE_MIN_WIDTH, LANE_MAX_WIDTH);
    let vert_road_width = 2 * rng.gen_range(LANE_MIN_WIDTH, LANE_MAX_WIDTH);
    let horiz_sidewalk_width = rng.gen_range(SIDEWALK_MIN_WIDTH, SIDEWALK_MAX_WIDTH);
    let vert_sidewalk_width = rng.gen_range(SIDEWALK_MIN_WIDTH, SIDEWALK_MAX_WIDTH);

    println!("horiz width: {} vert width: {}", horiz_road_width, vert_road_width);
    println!("horiz sidewalk width: {} vert sidewalk width: {}", horiz_sidewalk_width, vert_sidewalk_width);

    let horiz_road_length_max = MAP_WIDTH - BUILDING_WIDTH * 2 - vert_sidewalk_width * 2;
    let vert_road_length_max = MAP_HEIGHT - BUILDING_WIDTH * 2 - horiz_sidewalk_width * 2;
    let horiz_road_min_length = (BUILDING_WIDTH + vert_sidewalk_width) * 2 + 2 * vert_road_width;
    let vert_road_min_length = (BUILDING_WIDTH + horiz_sidewalk_width) * 2 + 2 * horiz_road_width;
    let horiz_road_length = rng.gen_range(horiz_road_min_length, horiz_road_length_max);
    let vert_road_length = rng.gen_range(vert_road_min_length, vert_road_length_max);

    // mark the grid accordingly
    // horizontal roads, both top and bottom
    {
        let left = BUILDING_WIDTH + vert_sidewalk_width;
        let top_1 = BUILDING_WIDTH + horiz_sidewalk_width;
        let top_2 = top_1 + vert_road_length - horiz_road_width;
        fill_map_rect(&mut grid[..], left, top_1,
                                     horiz_road_length, horiz_road_width/2,
                                     Cell::OUTER_LANE);
        fill_map_rect(&mut grid[..], left + vert_road_width/2, top_2,
                                     horiz_road_length - vert_road_width, horiz_road_width/2,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut grid[..], left + vert_road_width/2, top_1 + horiz_road_width/2,
                                     horiz_road_length - vert_road_width, horiz_road_width/2,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut grid[..], left, top_2 + horiz_road_width/2,
                                     horiz_road_length, horiz_road_width/2,
                                     Cell::OUTER_LANE);
    }

    // vertical roads
    {
        let top = BUILDING_WIDTH + horiz_sidewalk_width;
        let left_1 = BUILDING_WIDTH + vert_sidewalk_width;
        let left_2 = left_1 + horiz_road_length - vert_road_width;
        fill_map_rect(&mut grid[..], left_1, top,
                                     vert_road_width/2, vert_road_length,
                                     Cell::OUTER_LANE);
        fill_map_rect(&mut grid[..], left_2, top + horiz_road_width/2,
                                     vert_road_width/2, vert_road_length - horiz_road_width,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut grid[..], left_1 + vert_road_width/2, top + horiz_road_width/2,
                                     vert_road_width/2, vert_road_length - horiz_road_width,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut grid[..], left_2 + vert_road_width/2, top,
                                     vert_road_width/2, vert_road_length,
                                     Cell::OUTER_LANE);
    }

    // buildings
    let outer_perim = horiz_road_length * 2 + vert_road_length * 2;
    let inner_perim = outer_perim - (vert_road_width * 2 + horiz_road_width * 2 + vert_sidewalk_width * 2 + horiz_sidewalk_width * 2) * 2;
    let total_perim = outer_perim + inner_perim;
    let mut chosen_perims = [0usize; BUILDING_N];
    for i in 0..BUILDING_N {
        let mut perim_loc;
        loop {
            perim_loc = rng.gen_range(0, total_perim);
            // check that existing buildings are far enough away
            if chosen_perims.iter().take(i).all(|p| (*p as i32 - perim_loc as i32).abs() > BUILDING_SPACING as i32) {
                break;
            }
        }
        chosen_perims[i] = perim_loc;

        // perim_loc goes through outer_perim then inner_perim
        // for each it goes top, right, bottom, left in that order.
        // println!("choose perim_loc: {}", perim_loc);
        if perim_loc < outer_perim {
            let left = BUILDING_WIDTH + vert_sidewalk_width;
            let top = BUILDING_WIDTH + horiz_sidewalk_width;
            let right = left + horiz_road_length + vert_sidewalk_width;
            let bottom = top + vert_road_length + horiz_sidewalk_width;
            if perim_loc < horiz_road_length {
                fill_map_rect(&mut grid[..], left + perim_loc, 0, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                buildings[i] = (left + perim_loc, 0);
                continue;
            }
            let perim_loc = perim_loc - horiz_road_length;
            if perim_loc < vert_road_length {
                fill_map_rect(&mut grid[..], right, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                buildings[i] = (right, top + perim_loc);
                continue;
            }
            let perim_loc = perim_loc - vert_road_length;
            if perim_loc < horiz_road_length {
                fill_map_rect(&mut grid[..], left + perim_loc, bottom, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                buildings[i] = (left + perim_loc, bottom);
                continue;
            }
            let perim_loc = perim_loc - horiz_road_length;
            fill_map_rect(&mut grid[..], 0, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
            buildings[i] = (0, top + perim_loc);
        } else {
            let perim_loc = perim_loc - outer_perim;
            let inner_h_length = horiz_road_length - 2 * vert_road_width - 2 * vert_sidewalk_width;
            let inner_v_length = vert_road_length - 2 * horiz_road_width - 2 * horiz_sidewalk_width;
            let left = BUILDING_WIDTH + 2 * vert_sidewalk_width + vert_road_width;
            let top = BUILDING_WIDTH + 2 * horiz_sidewalk_width + horiz_road_width;
            let right = horiz_road_length - vert_road_width;
            let bottom = vert_road_length - horiz_road_width;
            if perim_loc < inner_h_length {
                fill_map_rect(&mut grid[..], left + perim_loc, top, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                buildings[i] = (left + perim_loc, top);
                continue;
            }
            let perim_loc = perim_loc - inner_h_length;
            if perim_loc < inner_v_length {
                fill_map_rect(&mut grid[..], right, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                buildings[i] = (right, top + perim_loc);
                continue;
            }
            let perim_loc = perim_loc - inner_v_length;
            if perim_loc < inner_h_length {
                fill_map_rect(&mut grid[..], left + perim_loc, bottom, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                buildings[i] = (left + perim_loc, bottom);
                continue;
            }
            let perim_loc = perim_loc - inner_h_length;
            fill_map_rect(&mut grid[..], left, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
            buildings[i] = (left, top + perim_loc);
        }
    }

    // crosswalks
    let horiz_crosswalk_perim = horiz_road_length - 2 * vert_road_width - CROSSWALK_WIDTH;
    let vert_crosswalk_perim = vert_road_length - 2 * horiz_road_width - CROSSWALK_WIDTH;
    let crosswalk_perim = horiz_crosswalk_perim * 2 + vert_crosswalk_perim * 2;
    let mut chosen_crosswalks = [0usize; CROSSWALK_N];
    for i in 0..CROSSWALK_N {
        let mut perim_loc;
        loop {
            perim_loc = rng.gen_range(0, crosswalk_perim);
            // check that existing buildings are far enough away
            if chosen_crosswalks.iter().take(i).all(|p| (*p as i32 - perim_loc as i32).abs() > CROSSWALK_SPACING as i32) {
                break;
            }
        }
        chosen_crosswalks[i] = perim_loc;

        // go through sections, top, right, bottom, left...
        let left1 = BUILDING_WIDTH + vert_sidewalk_width;
        let left2 = BUILDING_WIDTH + vert_sidewalk_width + vert_road_width;
        let top1 = BUILDING_WIDTH + horiz_sidewalk_width;
        let top2 = BUILDING_WIDTH + horiz_sidewalk_width + horiz_road_width;
        let right = left1 + horiz_road_length - vert_road_width;
        let bottom = top1 + vert_road_length - horiz_road_width;
        if perim_loc < horiz_crosswalk_perim {
            fill_map_rect(&mut grid[..], left2 + perim_loc, top1, CROSSWALK_WIDTH, horiz_road_width, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - horiz_crosswalk_perim;
        if perim_loc < vert_crosswalk_perim {
            fill_map_rect(&mut grid[..], right, top2 + perim_loc, vert_road_width, CROSSWALK_WIDTH, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - vert_crosswalk_perim;
        if perim_loc < horiz_crosswalk_perim {
            fill_map_rect(&mut grid[..], left2 + perim_loc, bottom, CROSSWALK_WIDTH, horiz_road_width, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - horiz_crosswalk_perim;
        fill_map_rect(&mut grid[..], left1, top2 + perim_loc, vert_road_width, CROSSWALK_WIDTH, Cell::CROSSWALK);
    }

    WorldMap{grid: Box::new(grid), buildings, grid_width: MAP_WIDTH}
}

#[derive(Clone, Copy, Debug, Default)]
struct Agent {
    on_map: bool,
    pose: (usize, usize, f32), // x, y, theta (rads)
    speed: usize,
    health: i32,
    has_deadline: bool,
    deadline_reward: f32,
    timestep_cost: f32,
    risk_cost: f32,
    folk_theory: f32,
    habit_theory: f32,
    internal_theory: f32,
    restricted_set: bool,
}

// we only serialize and send the pose (and whether agent is on map)
impl serde::Serialize for Agent {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where S: serde::Serializer
    {
        let mut seq = serializer.serialize_seq(Some(3))?;
        seq.serialize_element(&self.on_map)?;
        seq.serialize_element(&self.pose.0)?;
        seq.serialize_element(&self.pose.1)?;
        seq.serialize_element(&self.pose.2)?;
        seq.end()
    }
}

#[derive(Clone, Serialize)]
struct WorldAgents {
    pedestrians: Box<[Agent]>,
    vehicles: Box<[Agent]>,
}

#[derive(Clone)]
struct WorldState {
    agents: Arc<Mutex<WorldAgents>>,
    map: WorldMap,
}

fn create_agents() -> WorldAgents {
    let mut pedestrians = [Agent::default(); POPULATION_N/2];
    let mut vehicles = [Agent::default(); POPULATION_N/2];

    WorldAgents { pedestrians: Box::new(pedestrians), vehicles: Box::new(vehicles) }
}

#[get("/")]
fn index(state: State<WorldState>) -> Template {
	Template::render("index", &state.map)
}

#[get("/agents")]
fn get_agents(state: State<WorldState>) -> Json<Value> {
	Json(json!(&*state.agents.lock().unwrap()))
}

#[get("/resources/<file..>")]
fn resources(file: PathBuf) -> Option<NamedFile> {
	NamedFile::open(Path::new("resources/").join(file)).ok()
}

fn main() {
    let map = create_map();
    let agents = create_agents();
    let state = WorldState {map, agents: Arc::new(Mutex::new(agents))};

    rocket::ignite()
        .manage(state.clone())
        .mount("/", routes![index, get_agents, resources])
        .attach(Template::fairing())
        .launch();
}

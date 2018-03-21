#![feature(plugin)]
#![feature(custom_derive)]
#![plugin(rocket_codegen)]

extern crate rocket;
#[macro_use] extern crate rocket_contrib;
use rocket_contrib::Template;
#[macro_use] extern crate bitflags;
extern crate rand;
#[macro_use] extern crate rand_derive;
extern crate serde;
extern crate serde_json;
#[macro_use] extern crate serde_derive;

use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::f32::{self, consts};
use std::thread;
use rocket::{State};
use rocket::response::{NamedFile};
use rocket_contrib::{Json, Value};
use rand::Rng;
use serde::ser::SerializeStruct;

mod general_search;
#[allow(unused_imports)]
use general_search::{PriorityQueue, SearchProblemTrait, SearchNode, create_search_problem, tree_search};

#[cfg(test)]
mod tests;

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

const MAP_WIDTH: usize = 74;
const MAP_HEIGHT: usize = 74;
const MAP_SIZE: usize = MAP_WIDTH * MAP_HEIGHT;
const BUILDING_N: usize = 30;
const CROSSWALK_N: usize = 6;
// const EXTRA_OBSTACLE_N: usize = 6;
const POPULATION_N: usize = 24; // one-half this for each of pedestrians and vehicles
const PEDESTRIAN_N: usize = 6; // those on map at one time
const VEHICLE_N: usize = 6;
const SPAWN_MARGIN: usize = 3; // clearance from other agents when spawning

const BUILDING_WIDTH: usize = 3; // just a facade
const BUILDING_SPACING: usize = 12;
const CROSSWALK_WIDTH: usize = 2;
const CROSSWALK_SPACING: usize = 6;
const PEDESTRIAN_SIZE: usize = 1;
const VEHICLE_WIDTH: usize = 4;
const VEHICLE_LENGTH: usize = 10;
const SIDEWALK_MIN_WIDTH: usize = 3;
const SIDEWALK_MAX_WIDTH: usize = 6;
const LANE_MIN_WIDTH: usize = 7;
const LANE_MAX_WIDTH: usize = 11;

const FROZEN_STEPS: usize = 6;
const ATTEMPTS: usize = 100;

// distance at which the pedestrian is considered adjacent to the building
const BUILDING_PEDESTRIAN_MARGIN: usize = 3;
// distance (besides crosswalk) at which the vehicle is considered adjacent to the building
const BUILDING_VEHICLE_MARGIN: usize = 6;

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

#[derive(Clone)]
struct WorldMap {
    grid: Box<[Cell]>,
    buildings: Vec<(usize, usize)>,
    grid_width: usize,
    horiz_road_width: usize,
    vert_road_width: usize,
    horiz_sidewalk_width: usize,
    vert_sidewalk_width: usize,
    horiz_road_length: usize,
    vert_road_length: usize,

    pedestrian_outer_pts: [(usize, usize); 4],
    pedestrian_inner_pts: [(usize, usize); 4],
    vehicle_outer_pts: [(usize, usize); 4],
    vehicle_inner_pts: [(usize, usize); 4],
}

impl serde::Serialize for WorldMap {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where S: serde::Serializer
    {
        // 3 is the number of fields in the struct.
        let mut state = serializer.serialize_struct("WorldMap", 3)?;
        state.serialize_field("grid", &self.grid)?;
        state.serialize_field("buildings", &self.buildings)?;
        state.serialize_field("grid_width", &self.grid_width)?;
        state.end()
    }
}

fn fill_map_rect(grid: &mut [Cell], left: usize, top: usize,
                                    width: usize, height: usize, mask: Cell) {
    // println!("left: {} top: {} width: {} height: {}", left, top, width, height);
    for y in top..(top + height) {
        let mut grid_i = y * MAP_WIDTH + left;
        for _ in 0..width {
            grid[grid_i] |= mask;
            grid_i += 1;
        }
    }
}

fn create_buildings(m: &mut WorldMap) {
    let mut rng = rand::thread_rng();

    let outer_perim = m.horiz_road_length * 2 + m.vert_road_length * 2;
    let inner_perim = outer_perim - (m.vert_road_width * 2 + m.horiz_road_width * 2 +
                                     m.vert_sidewalk_width * 2 + m.horiz_sidewalk_width * 2 +
                                     (BUILDING_WIDTH - 1) * 2
                                    ) * 2;
    let total_perim = outer_perim + inner_perim;
    let mut chosen_perims = [0usize; BUILDING_N];

    let mut ordering = (0..total_perim).collect::<Vec<_>>();
    rng.shuffle(&mut ordering);
    let mut next_ordering_i = 0;

    for i in 0..BUILDING_N {
        let mut perim_loc = 99999;
        for &perim_i in ordering.iter().skip(next_ordering_i) {
            // check that existing buildings are far enough away
            // only compare with those on the correct inner/outer path
            if perim_i < outer_perim {
                if chosen_perims.iter().take(i).all(|&p|
                        p >= outer_perim ||
                        mod_dist(p, perim_i, outer_perim) > BUILDING_SPACING) {
                    perim_loc = perim_i;
                    next_ordering_i += 1;
                    break;
                }
            } else {
                if chosen_perims.iter().take(i).all(|&p|
                        p < outer_perim ||
                        mod_dist(p - outer_perim, perim_i - outer_perim, inner_perim)
                            > BUILDING_SPACING) {
                    perim_loc = perim_i;
                    next_ordering_i += 1;
                    break;
                }
            }
            next_ordering_i += 1;
        }
        if next_ordering_i >= ordering.len() {
            return; // nothing left to check
        }
        // println!("next_ordering_i: {} ordering.len(): {}", next_ordering_i, ordering.len());
        chosen_perims[i] = perim_loc;
        // println!("Choose building loc: {}", perim_loc);

        // perim_loc goes through outer_perim then inner_perim
        // for each it goes top, right, bottom, left in that order.
        // println!("choose perim_loc: {}", perim_loc);
        if perim_loc < outer_perim {
            let left = BUILDING_WIDTH + m.vert_sidewalk_width;
            let top = BUILDING_WIDTH + m.horiz_sidewalk_width;
            let right = left + m.horiz_road_length + m.vert_sidewalk_width;
            let bottom = top + m.vert_road_length + m.horiz_sidewalk_width;
            if perim_loc < m.horiz_road_length {
                fill_map_rect(&mut m.grid[..], left + perim_loc, 0, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                m.buildings.push((left + perim_loc, 0));
                continue;
            }
            let perim_loc = perim_loc - m.horiz_road_length;
            if perim_loc < m.vert_road_length {
                fill_map_rect(&mut m.grid[..], right, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                m.buildings.push((right, top + perim_loc));
                continue;
            }
            let perim_loc = perim_loc - m.vert_road_length;
            if perim_loc < m.horiz_road_length {
                fill_map_rect(&mut m.grid[..], left + perim_loc, bottom, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                m.buildings.push((left + perim_loc, bottom));
                continue;
            }
            let perim_loc = perim_loc - m.horiz_road_length;
            fill_map_rect(&mut m.grid[..], 0, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
            m.buildings.push((0, top + perim_loc));
        } else {
            let perim_loc = perim_loc - outer_perim;
            let inner_h_length = m.horiz_road_length - 2 * m.vert_road_width - 2 * m.vert_sidewalk_width - (BUILDING_WIDTH - 1);
            let inner_v_length = m.vert_road_length - 2 * m.horiz_road_width - 2 * m.horiz_sidewalk_width - (BUILDING_WIDTH - 1);
            let left = BUILDING_WIDTH + 2 * m.vert_sidewalk_width + m.vert_road_width;
            let top = BUILDING_WIDTH + 2 * m.horiz_sidewalk_width + m.horiz_road_width;
            let right = m.horiz_road_length - m.vert_road_width;
            let bottom = m.vert_road_length - m.horiz_road_width;
            if perim_loc < inner_h_length {
                fill_map_rect(&mut m.grid[..], left + perim_loc, top, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                m.buildings.push((left + perim_loc, top));
                continue;
            }
            let perim_loc = perim_loc - inner_h_length;
            if perim_loc < inner_v_length {
                fill_map_rect(&mut m.grid[..], right, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                m.buildings.push((right, top + perim_loc));
                continue;
            }
            let perim_loc = perim_loc - inner_v_length;
            if perim_loc < inner_h_length {
                fill_map_rect(&mut m.grid[..], left + perim_loc, bottom, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
                m.buildings.push((left + perim_loc, bottom));
                continue;
            }
            let perim_loc = perim_loc - inner_h_length;
            fill_map_rect(&mut m.grid[..], left, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTALCE);
            m.buildings.push((left, top + perim_loc));
        }
    }
}

fn create_crosswalks(m: &mut WorldMap) {
    let mut rng = rand::thread_rng();

    let horiz_crosswalk_perim = m.horiz_road_length - 2 * m.vert_road_width - CROSSWALK_WIDTH;
    let vert_crosswalk_perim = m.vert_road_length - 2 * m.horiz_road_width - CROSSWALK_WIDTH;
    let crosswalk_perim = horiz_crosswalk_perim * 2 + vert_crosswalk_perim * 2;
    let mut chosen_crosswalks = [0usize; CROSSWALK_N];
    for i in 0..CROSSWALK_N {
        let mut perim_loc;
        let mut attempts_i = 0;
        loop {
            perim_loc = rng.gen_range(0, crosswalk_perim);
            // check that existing crosswalks are far enough away
            if chosen_crosswalks.iter().take(i).all(|p| (*p as i32 - perim_loc as i32).abs() > CROSSWALK_SPACING as i32) {
                break;
            }
            attempts_i += 1;
            if attempts_i > ATTEMPTS {
                return; // probably constrained to not get all of them
            }
        }
        chosen_crosswalks[i] = perim_loc;

        // go through sections, top, right, bottom, left...
        let left1 = BUILDING_WIDTH + m.vert_sidewalk_width;
        let left2 = BUILDING_WIDTH + m.vert_sidewalk_width + m.vert_road_width;
        let top1 = BUILDING_WIDTH + m.horiz_sidewalk_width;
        let top2 = BUILDING_WIDTH + m.horiz_sidewalk_width + m.horiz_road_width;
        let right = left1 + m.horiz_road_length - m.vert_road_width;
        let bottom = top1 + m.vert_road_length - m.horiz_road_width;
        if perim_loc < horiz_crosswalk_perim {
            fill_map_rect(&mut m.grid[..], left2 + perim_loc, top1, CROSSWALK_WIDTH, m.horiz_road_width, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - horiz_crosswalk_perim;
        if perim_loc < vert_crosswalk_perim {
            fill_map_rect(&mut m.grid[..], right, top2 + perim_loc, m.vert_road_width, CROSSWALK_WIDTH, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - vert_crosswalk_perim;
        if perim_loc < horiz_crosswalk_perim {
            fill_map_rect(&mut m.grid[..], left2 + perim_loc, bottom, CROSSWALK_WIDTH, m.horiz_road_width, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - horiz_crosswalk_perim;
        fill_map_rect(&mut m.grid[..], left1, top2 + perim_loc, m.vert_road_width, CROSSWALK_WIDTH, Cell::CROSSWALK);
    }
}

fn create_map() -> WorldMap {
    // The world will have a single block of road, making a rectangle.
    let grid = [Cell{bits: 0u8}; MAP_SIZE];
    let buildings = Vec::new();

    let mut rng = rand::thread_rng();

    // we make the horizontal (at top and bottom) roads have the same parameters
    // and the vertical ones have their own
    let horiz_road_width = 2 * rng.gen_range(LANE_MIN_WIDTH, LANE_MAX_WIDTH);
    let vert_road_width = 2 * rng.gen_range(LANE_MIN_WIDTH, LANE_MAX_WIDTH);
    let horiz_sidewalk_width = rng.gen_range(SIDEWALK_MIN_WIDTH, SIDEWALK_MAX_WIDTH);
    let vert_sidewalk_width = rng.gen_range(SIDEWALK_MIN_WIDTH, SIDEWALK_MAX_WIDTH);

    let horiz_road_length_max = MAP_WIDTH - BUILDING_WIDTH * 2 - vert_sidewalk_width * 2;
    let vert_road_length_max = MAP_HEIGHT - BUILDING_WIDTH * 2 - horiz_sidewalk_width * 2;
    let horiz_road_min_length = (BUILDING_WIDTH + vert_sidewalk_width) * 2 + 2 * vert_road_width;
    let vert_road_min_length = (BUILDING_WIDTH + horiz_sidewalk_width) * 2 + 2 * horiz_road_width;
    let horiz_road_length = rng.gen_range(horiz_road_min_length, horiz_road_length_max);
    let vert_road_length = rng.gen_range(vert_road_min_length, vert_road_length_max);

    println!("\nhoriz length: {} vert length: {}", horiz_road_length, vert_road_length);
    println!("horiz width: {} vert width: {}", horiz_road_width, vert_road_width);
    println!("horiz sidewalk width: {} vert sidewalk width: {}", horiz_sidewalk_width, vert_sidewalk_width);

    let mut map = WorldMap{grid: Box::new(grid), buildings, grid_width: MAP_WIDTH,
                             horiz_road_width, vert_road_width,
                             horiz_sidewalk_width, vert_sidewalk_width,
                             horiz_road_length, vert_road_length,
                             pedestrian_inner_pts: [(0, 0); 4],
                             pedestrian_outer_pts: [(0, 0); 4],
                             vehicle_inner_pts: [(0, 0); 4],
                             vehicle_outer_pts: [(0, 0); 4]};

    // mark the grid accordingly
    // horizontal roads, both top and bottom
    {
        let left = BUILDING_WIDTH + vert_sidewalk_width;
        let top_1 = BUILDING_WIDTH + horiz_sidewalk_width;
        let top_2 = top_1 + vert_road_length - horiz_road_width;
        fill_map_rect(&mut map.grid[..], left, top_1,
                                     horiz_road_length, horiz_road_width/2,
                                     Cell::OUTER_LANE);
        fill_map_rect(&mut map.grid[..], left + vert_road_width/2, top_2,
                                     horiz_road_length - vert_road_width, horiz_road_width/2,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut map.grid[..], left + vert_road_width/2, top_1 + horiz_road_width/2,
                                     horiz_road_length - vert_road_width, horiz_road_width/2,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut map.grid[..], left, top_2 + horiz_road_width/2,
                                     horiz_road_length, horiz_road_width/2,
                                     Cell::OUTER_LANE);
    }

    // vertical roads
    {
        let top = BUILDING_WIDTH + horiz_sidewalk_width;
        let left_1 = BUILDING_WIDTH + vert_sidewalk_width;
        let left_2 = left_1 + horiz_road_length - vert_road_width;
        fill_map_rect(&mut map.grid[..], left_1, top,
                                     vert_road_width/2, vert_road_length,
                                     Cell::OUTER_LANE);
        fill_map_rect(&mut map.grid[..], left_2, top + horiz_road_width/2,
                                     vert_road_width/2, vert_road_length - horiz_road_width,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut map.grid[..], left_1 + vert_road_width/2, top + horiz_road_width/2,
                                     vert_road_width/2, vert_road_length - horiz_road_width,
                                     Cell::INNER_LANE);
        fill_map_rect(&mut map.grid[..], left_2 + vert_road_width/2, top,
                                     vert_road_width/2, vert_road_length,
                                     Cell::OUTER_LANE);
    }

    create_buildings(&mut map);
    create_crosswalks(&mut map);

    map
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
enum AgentKind {
    Obstacle,
    Pedestrian,
    Vehicle,
}

impl Default for AgentKind {
    fn default() -> AgentKind { AgentKind::Obstacle }
}

#[derive(Clone, Copy, Debug, Default)]
struct Agent {
    kind: AgentKind,
    on_map: bool,
    frozen_steps: usize, // does not get an action for x steps... like after a collision
    pose: (usize, usize, f32), // x, y, theta (rads)
    width: usize, // perpendicular to theta
    length: usize, // in direction of theta
    velocity: isize,
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
        let mut state = serializer.serialize_struct("Agent", 8)?;
        state.serialize_field("kind", &self.kind)?;
        state.serialize_field("on_map", &self.on_map)?;
        state.serialize_field("x", &self.pose.0)?;
        state.serialize_field("y", &self.pose.1)?;
        state.serialize_field("theta", &self.pose.2)?;
        state.serialize_field("width", &self.width)?;
        state.serialize_field("length", &self.length)?;
        state.serialize_field("health", &self.health)?;
        state.end()
    }
}

#[derive(Clone)]
struct WorldState {
    agents: Arc<Mutex<Vec<Agent>>>,
    map: WorldMap,
}

type LineUsize = ((usize, usize), (usize, usize));
type LineF32 = ((f32, f32), (f32, f32));
type RectUsize = [(usize, usize); 4];
type RectF32 = [(f32, f32); 4];

fn points_to_lines_f32(pts: &RectF32) -> Vec<LineF32> {
    pts.iter().cloned().zip(pts.iter().skip(1).chain(pts.iter().take(1)).cloned()).collect()
}

fn points_to_lines_usize(pts: &RectUsize) -> Vec<LineUsize> {
    pts.iter().cloned().zip(pts.iter().skip(1).chain(pts.iter().take(1)).cloned()).collect()
}

fn points_usize_to_lines_f32(pts: &[(usize, usize); 4]) -> Vec<LineF32> {
    let pts1 = pts.iter().map(|&(x, y)| (x as f32, y as f32));
    let pts2 = pts.iter().skip(1).chain(pts.iter().take(1)).map(|&(x, y)| (x as f32, y as f32));
    pts1.zip(pts2).collect()
}

// manhattan distance, the 1 norm
// fn dist_1(pts: ((usize, usize), (usize, usize))) -> usize {
//     let ((x1, y1), (x2, y2)) = pts;
//     return max(x1, x2) - min(x1, x2) + max(y1, y2) - min(y1, y2);
// }

fn dist_1<T>(pts: ((T, T), (T, T))) -> T
    where T: std::ops::Add<Output=T> + std::ops::Sub<Output=T> + PartialOrd
{
    let ((x1, y1), (x2, y2)) = pts;
    return if x1 > x2 { x1 - x2 } else { x2 - x1 } + if y1 > y2 { y1 - y2 } else { y2 - y1 };
}

fn difference<T>(a: T, b: T) -> T
    where T: std::ops::Add<Output=T> + std::ops::Sub<Output=T> + PartialOrd
{
    if a > b { a - b } else  { b - a }
}

fn pose_size_to_bounding_rect(p: (usize, usize, f32), size: (usize, usize))
                              -> ((usize, usize), (usize, usize)) {
    let (x, y, t) = p;
    let (sx, sy) = size;

    let rw = rotate_pt((sx as f32, 0.0), -t);
    let rl = rotate_pt((0.0, sy as f32), -t);
    let (x, y) = (x as f32, y as f32);

    let min_x = x.min(x + rw.0).min(x + rl.0).min(x + rw.0 + rl.0).floor().max(0.0) as usize;
    let min_y = y.min(y + rw.1).min(y + rl.1).min(y + rw.1 + rl.1).floor().max(0.0) as usize;
    let max_x = x.max(x + rw.0).max(x + rl.0).max(x + rw.0 + rl.0).ceil().max(0.0) as usize;
    let max_y = y.max(y + rw.1).max(y + rl.1).max(y + rw.1 + rl.1).ceil().max(0.0) as usize;

    ((min_x, min_y), (max_x, max_y))
}

// manhattan distance/1 norm, considers that the positions are upper-left corners
// and that the objects have their sizes expanding to the right and down.
// uses the bounding box of the rotated objects since this is mainly intended for
// dealing only with rotations that are multiples of 90 degrees.
// xxx yyy = 1
// xxx
//   yyy = 0 (-1)
fn obj_dist_1(a: (usize, usize, f32), size_a: (usize, usize),
              b: (usize, usize, f32), size_b: (usize, usize)) -> usize
{
    // l for low (min) and h for high (max)
    let ((lx1, ly1), (hx1, hy1)) = pose_size_to_bounding_rect(a, size_a);
    let ((lx2, ly2), (hx2, hy2)) = pose_size_to_bounding_rect(b, size_b);
    let x_diff = if lx1 < lx2 {
        if hx1 < lx2 { lx2 - hx1 } else { 0 }
    } else {
        if hx2 < lx1 { lx1 - hx2 } else { 0 }
    };
    let y_diff = if ly1 < ly2 {
        if hy1 < ly2 { ly2 - hy1 } else { 0 }
    } else {
        if hy2 < ly1 { ly1 - hy2 } else { 0 }
    };
    x_diff + y_diff
}

fn calc_total_perim(pts: &RectUsize) -> usize {
    let lines = points_to_lines_usize(pts);
    lines.into_iter().map(|a| dist_1(a)).sum()
}

fn perim_i_to_pose(m: &WorldMap, perim_i: usize, perims: &Vec<usize>, lines: &Vec<LineUsize>,
                   size: (usize, usize), is_inner: bool) -> Option<(usize, usize, f32)> {
    let perim_total = perims.iter().sum();
    if perim_i >= perim_total {
        panic!("Perim_i {} is invalid, exceeds perimeter total {}", perim_i, perim_total);
    }

    let mut perim_val = perim_i;
    let mut line_i = 0;
    for (i, &perim) in perims.iter().enumerate() {
        if perim_val < perim {
            line_i = i;
            break;
        }
        perim_val -= perim;
    }

    // first get exact point on the perimeter
    let ((x1, y1), (x2, y2)) = lines[line_i];
    let sx = (x2 as i32 - x1 as i32).signum();
    let sy = (y2 as i32 - y1 as i32).signum();
    let pt = ((x1 as i32 + sx * perim_val as i32) as usize,
              (y1 as i32 + sy * perim_val as i32) as usize);
    // include the boundary such that results look like they are from [min, max)
    // even when we actually iterate from high to low
    let pt = (pt.0 - if sx == -1 { 1 } else { 0 },
              pt.1 - if sy == -1 { 1 } else { 0 });

    // transform that initial point to a pose that takes into account
    // size of the object and its orientation (based on the line it is on)
    if is_inner {
        if line_i == 0 {
            if pt.0 < x1 + size.1 - m.vert_road_width / 2 {
                return None;
            }
            return Some((pt.0 + 1, pt.1 - size.0, -consts::PI / 2.0));
        } else if line_i == 1 {
            if pt.1 < y1 + size.1 - m.horiz_road_width / 2 {
                return None;
            }
            return Some((pt.0 + size.0, pt.1 + 1, consts::PI));
        } else if line_i == 2 {
            if pt.0 > x1 - size.1 + m.vert_road_width / 2 {
                return None;
            }
            return Some((pt.0, pt.1 + size.0, consts::PI / 2.0));
        } else if line_i == 3 {
            if pt.1 > y1 - size.1 + m.horiz_road_width / 2 {
                return None;
            }
            return Some((pt.0 - size.0, pt.1, 0.0));
        } else {
            panic!("Should not be possible");
        }
    } else {
        if line_i == 0 {
            if pt.0 > x2 - size.1 {
                return None;
            }
            return Some((pt.0, pt.1 + size.0, consts::PI / 2.0));
        } else if line_i == 1 {
            if pt.1 > y2 - size.1 {
                return None;
            }
            return Some((pt.0 - size.0, pt.1, 0.0));
        } else if line_i == 2 {
            if pt.0 < x2 + size.1 {
                return None;
            }
            return Some((pt.0, pt.1 - size.0, -consts::PI / 2.0));
        } else if line_i == 3 {
            if pt.1 < y2 + size.1 {
                return None;
            }
            return Some((pt.0 + size.0, pt.1, consts::PI));
        } else {
            panic!("Should not be possible");
        }
    }
}

// tries to find a "spawn" pose (x, y, theta) for agent with size (width, length)
// within building_margin of a building along one of the two perimeters,
// margin away from all existing agents
// makes the assumption that points form a strict rectangle (only y or x change at a time)
fn spawn_from_perim(m: &WorldMap, building: (usize, usize), building_margin: usize,
                    size: (usize, usize), pts: &RectUsize, occupied: &Vec<bool>,
                    margin: usize, is_inner: bool)
                    -> Option<(usize, usize, f32)> {
    let mut rng = rand::thread_rng();

    let lines = points_to_lines_usize(pts);

    let perims = lines.iter().map(|&a| dist_1(a));
    let perims = perims.collect::<Vec<_>>();
    let perim_total = perims.iter().sum();

    let perim_loc;
    {
        let perim_choices = 0..perim_total;
        let margin = margin + size.1; // include agent length in margin
        let perim_choices = perim_choices.filter(|&i|
                                a_to_b_mod(i + perim_total - margin, i + margin + 1, perim_total)
                                .iter().all(|&j| !occupied[j]));
        let perim_choices = perim_choices.filter(|&i| {
            let pose = perim_i_to_pose(m, i, &perims, &lines, size, is_inner);
            if pose.is_none() {
                return false;
            }
            let pose = pose.unwrap();
            obj_dist_1((building.0, building.1, 0.0),
                       (BUILDING_WIDTH, BUILDING_WIDTH), pose, size) <= building_margin
        });
        let perim_choices = perim_choices.collect::<Vec<_>>();

        // let mut min_d = usize::max_value();
        // for i in 0..perim_total {
        //     let pt = perim_i_to_pt(i, &perims, &lines);
        //     // let m = existing.iter().map(|&b| ((i as i32) - (b as i32)).abs()).min();
        //     let d = obj_dist_1(building, (BUILDING_WIDTH, BUILDING_WIDTH),
        //                        perim_i_to_pt(i, &perims, &lines), size);
        //     min_d = min_d.min(d);
        //     // println!("Dist {} between {:?} and {:?} w/ margin {:?}", d, building, pt, m);
        // }
        // println!("Min Dist {} to {:?}", min_d, building);

        perim_loc = *rng.choose(&perim_choices)?;
    }

    let pose = perim_i_to_pose(m, perim_loc, &perims, &lines, size, is_inner)?;
    let dist = obj_dist_1((building.0, building.1, 0.0),
                          (BUILDING_WIDTH, BUILDING_WIDTH), pose, size);

    println!("Chose perim_loc: {} at dist {} from building {:?}, with pose {:?}",
             perim_loc, dist, building, pose);

    Some(pose)
}

// tries to find a "spawn" pose (x, y, theta) for agent with size (width, length)
// within building_margin of a building along one of the two perimeters,
// margin away from all existing agents
fn spawn_on_two_perims(m: &WorldMap, buildings: &[(usize, usize)], building_margin: usize,
                       size: (usize, usize),
                       pts_a: &RectUsize, pts_b: &RectUsize,
                       occupied_a: &Vec<bool>, occupied_b: &Vec<bool>,
                       margin: usize) -> Option<(usize, usize, f32)> {
    let mut rng = rand::thread_rng();

    let mut ordering = (0..buildings.len()).collect::<Vec<_>>();
    rng.shuffle(&mut ordering);
    for i in ordering {
        let building = buildings[i];

        let mut res = spawn_from_perim(m, building, building_margin, size,
                                       pts_a, occupied_a, margin, false);
        if res.is_none() {
            res = spawn_from_perim(m, building, building_margin, size,
                                   pts_b, occupied_b, margin, true);
        }
        if res.is_some() {
            return res;
        }
    }
    None
}

fn lines_to_normals(lines: &Vec<LineF32>) -> Vec<f32> {
    lines.iter().map(|ln| ((ln.1).1 - (ln.0).1).atan2((ln.1).0 - (ln.0).0)).collect()
}

fn rotate_pt(pt: (f32, f32), angle: f32) -> (f32, f32) {
    let (x, y) = (pt.0, pt.1);
    let (sin_a, cos_a) = angle.sin_cos();
    (cos_a * x - sin_a * y, sin_a * x + cos_a * y)
}

// project rectangle onto axis defined by angle and then return (min, max)
fn project_rect(rect: &RectF32, angle: f32) -> (f32, f32) {
    let projected = rect.iter().map(|&p| rotate_pt(p, angle).0).collect::<Vec<_>>();
    (projected.iter().fold(f32::MAX, |a, &b| a.min(b)),
     projected.iter().fold(f32::MIN, |a, &b| a.max(b)))
}

fn overlaps_rect(q1: &RectF32, q2: &RectF32) -> bool {
    // apply the separating axis theorem
    // we loops through the 2 axes of q1, then those of q2
    let q1_lines = points_to_lines_f32(q1);
    let normals1 = lines_to_normals(&q1_lines);
    let q2_lines = points_to_lines_f32(q2);
    let normals2 = lines_to_normals(&q2_lines);
    let normals = normals1.into_iter().chain(normals2.into_iter()).collect::<Vec<_>>();
    for normal in normals {
        let (min1, max1) = project_rect(q1, -normal);
        let (min2, max2) = project_rect(q2, -normal);
        // include epsilon value for rounding/imprecision error.
        // so that adjacent things are not considered overlapping
        if max1 <= (min2 + 1e-4) || max2 <= (min1 + 1e-4) {
            return false;
        }
    }

    true
}

fn agent_to_rect32(agent: &Agent) -> RectF32 {
    let (width, length) = (agent.width as f32, agent.length as f32);
    let (x, y, theta) = (agent.pose.0 as f32, agent.pose.1 as f32, agent.pose.2);
    let rot_width = rotate_pt((width, 0.0), -theta);
    let rot_length = rotate_pt((0.0, length), -theta);
    let agent_rect = [(x, y),
                      (x + rot_width.0, y + rot_width.1),
                      (x + rot_width.0 + rot_length.0, y + rot_width.1 + rot_length.1),
                      (x + rot_length.0, y + rot_length.1)];
    agent_rect
}

// if there is an overlap returns the approximate perimeter location of the overlap.
// as elsewhere, this goes through the perimeter clockwise top, right, bottom, left
fn overlaps_path(agent: &Agent, path: &RectUsize,
                 path_width: usize, is_inner: bool) -> Vec<usize> {
    let path_width = path_width as f32;
    let agent_rect = agent_to_rect32(agent);

    let mut overlap_locs = Vec::new();
    let path_lines = points_usize_to_lines_f32(path);
    // println!("len: {}", path_lines.len());
    let mut perim_loc = 0;
    for (i, &line) in path_lines.iter().enumerate() {
        let ((x1, y1), (x2, y2)) = line;
        let line_rect;
        let x1_eq_x2 = (x1 - x2).abs() < 1e-4;
        if x1_eq_x2 {
            let path_width =
                if (i == 1 && !is_inner) ||
                   (i == 3 && is_inner) {
                    -path_width
                } else {
                    path_width
                };
            line_rect = [(x1, y1),
                         (x1 + path_width, y1),
                         (x2 + path_width, y2),
                         (x2, y2)];
        } else { // y1 == y2
            let path_width =
                if (i == 0 && is_inner) ||
                   (i == 2 && !is_inner) {
                    -path_width
                } else {
                    path_width
                };
            line_rect = [(x1, y1),
                         (x1, y1 + path_width),
                         (x2, y2 + path_width),
                         (x2, y2)];
        }
        // println!("Path line rect {}: {:?}", i, line_rect);
        if overlaps_rect(&agent_rect, &line_rect) {
            // println!("Overlaps");
            let (x, y) = (agent.pose.0 as f32, agent.pose.1 as f32);
            let perim_pos = if x1_eq_x2 { (y - y1).abs() } else { (x - x1).abs() };
            overlap_locs.push(perim_loc + perim_pos as usize);
        }
        perim_loc += dist_1(line) as usize;
    }

    overlap_locs
}

// range like [a, b), but can go by +1 or -1
fn a_to_b(a: usize, b: usize) -> Vec<usize> {
    if a < b {
        return (a..b).collect();
    } else {
        return ((b+1)..(a+1)).rev().collect();
    }
}

// range like [a, b), but can wraps around modular boundary if a > b
fn a_to_b_mod(a: usize, b: usize, n: usize) -> Vec<usize> {
    let a = a % n;
    let b = b % n;
    if a < b {
        return (a..b).collect();
    } else {
        return (a..n).chain(0..b).collect();
    }
}

fn mod_dist(a: usize, b: usize, n: usize) -> usize {
    // let a = a % n;
    // let b = b % n;
    difference((a + n * 3 / 2 - b) % n, n / 2)
}

// assumes path is in order of top, right, bottom, left.
fn calc_occupied_perim_map(agents: &[Agent], path: &RectUsize,
                           path_width: usize, is_inner: bool) -> Vec<bool>
{
    let path_lines = points_to_lines_usize(path);
    let perim_total = calc_total_perim(path);
    let mut occupied_perims = vec![false; perim_total];

    let path_width = path_width as f32;

    let mut path_cell_rects = Vec::new();
    for (i, &line) in path_lines.iter().enumerate() {
        let ((x1, y1), (x2, y2)) = line;
        if x1 == x2 {
            let path_width =
                if (i == 1 && !is_inner) ||
                   (i == 3 && is_inner) {
                    -path_width
                } else {
                    path_width
                };
            let x = x1 as f32;
            for &y in a_to_b(y1, y2).iter() {
                // include the boundary such that results look like they are from [min, max)
                // even when we actually iterate from high to low
                let y = y - if y2 < y1 { 1 } else { 0 };
                let cell_rect = [(x, y as f32),
                                 (x + path_width, y as f32),
                                 (x + path_width, (y + 1) as f32),
                                 (x, (y + 1) as f32)];
                path_cell_rects.push(cell_rect);
            }
        } else { // y1 == y2
            let path_width =
                if (i == 0 && is_inner) ||
                   (i == 2 && !is_inner) {
                    -path_width
                } else {
                    path_width
                };
            let y = y1 as f32;
            for &x in a_to_b(x1, x2).iter() {
                // include the boundary such that results look like they are from [min, max)
                // even when we actually iterate from high to low
                let x = x - if x2 < x1 { 1 } else { 0 };
                let cell_rect = [(x as f32, y),
                                 (x as f32, y + path_width),
                                 ((x + 1) as f32, y + path_width),
                                 ((x + 1) as f32, y)];
                path_cell_rects.push(cell_rect);
            }
        }
    }

    for agent in agents {
        if !agent.on_map {
            continue;
        }

        let overlap_locs = overlaps_path(agent, path, path_width as usize, is_inner);
        if overlap_locs.len() == 0 {
            continue;
        }

        let agent_rect = agent_to_rect32(agent);
        // println!("Examining agent with rect: {:?}", agent_rect);

        // we only need to check along the path as far as this maximum dimension squared
        // from the overlap location calculated above.
        let agent_max_dim_sq = agent.width.pow(2) + agent.length.pow(2);

        for (perim_loc, cell_rect) in path_cell_rects.iter().enumerate() {
            if occupied_perims[perim_loc] {
                continue;
            }

            let mut should_skip = true;
            for &overlap_loc in &overlap_locs {
                let overlap_loc_dist_sq = mod_dist(perim_loc, overlap_loc, perim_total).pow(2);
                if overlap_loc_dist_sq <= agent_max_dim_sq {
                    should_skip = false;
                    break;
                }
                // println!("Vote to skip perim_loc {} with mod_dist_sq {} < than {}", perim_loc, overlap_loc_dist_sq, agent_max_dim_sq);
            }
            if should_skip {
                continue;
            }

            if overlaps_rect(&agent_rect, &cell_rect) {
                occupied_perims[perim_loc] = true;
            //     println!("Overlaps perim_loc {} w/ cell rect {:?}", perim_loc, cell_rect);
            // } else {
            //     println!("Does not overlap perim_loc {} w/ cell rect {:?}", perim_loc, cell_rect);
            }
        }
    }
    occupied_perims
}

fn draw_off_map_agent_i(agents: &[Agent], kind: AgentKind) -> Option<usize> {
    let mut rng = rand::thread_rng();
    let off_map_agents = agents.iter()
                               .enumerate()
                               .filter_map(|(i, a)|
                                    if a.kind == kind && !a.on_map { Some(i) } else { None })
                               .collect::<Vec<_>>();
    if off_map_agents.len() == 0 {
        return None
    }
    let i = off_map_agents[rng.gen_range(0, off_map_agents.len())];
    Some(i)
}

fn setup_map_paths(m: &mut WorldMap) {
    // path that pedestrians spawn on (crosswalk by buildings)
    {
        let left_outer = BUILDING_WIDTH + m.vert_sidewalk_width / 2;
        let top_outer = BUILDING_WIDTH + m.horiz_sidewalk_width / 2;
        let right_outer = BUILDING_WIDTH + m.vert_sidewalk_width * 3 / 2 + m.horiz_road_length;
        let bottom_outer = BUILDING_WIDTH + m.horiz_sidewalk_width * 3 / 2 + m.vert_road_length;
        let pts_outer = [(left_outer, top_outer),
                         (right_outer, top_outer),
                         (right_outer, bottom_outer),
                         (left_outer, bottom_outer)];

        let left_inner = BUILDING_WIDTH + m.vert_sidewalk_width * 3 / 2 + m.vert_road_width;
        let top_inner = BUILDING_WIDTH + m.horiz_sidewalk_width * 3 / 2 + m.horiz_road_width;
        let right_inner = right_outer - m.vert_road_width - m.vert_sidewalk_width;
        let bottom_inner = bottom_outer - m.horiz_road_width - m.horiz_sidewalk_width;
        let pts_inner = [(left_inner, top_inner),
                         (right_inner, top_inner),
                         (right_inner, bottom_inner),
                         (left_inner, bottom_inner)];

        m.pedestrian_outer_pts = pts_outer;
        m.pedestrian_inner_pts = pts_inner;
    }
    // path that vehicles spawn on (side of road by sidewalk)
    {
        let left_outer = BUILDING_WIDTH + m.vert_sidewalk_width;
        let top_outer = BUILDING_WIDTH + m.horiz_sidewalk_width;
        let right_outer = BUILDING_WIDTH + m.vert_sidewalk_width + m.horiz_road_length;
        let bottom_outer = BUILDING_WIDTH + m.horiz_sidewalk_width + m.vert_road_length;
        let pts_outer = [(left_outer, top_outer),
                         (right_outer, top_outer),
                         (right_outer, bottom_outer),
                         (left_outer, bottom_outer)];

        let left_inner = BUILDING_WIDTH + m.vert_sidewalk_width + m.vert_road_width;
        let top_inner = BUILDING_WIDTH + m.horiz_sidewalk_width + m.horiz_road_width;
        let right_inner = BUILDING_WIDTH + m.vert_sidewalk_width + m.horiz_road_length - m.vert_road_width;
        let bottom_inner = BUILDING_WIDTH + m.horiz_sidewalk_width + m.vert_road_length - m.horiz_road_width;
        let pts_inner = [(left_inner, top_inner),
                         (right_inner, top_inner),
                         (right_inner, bottom_inner),
                         (left_inner, bottom_inner)];

        m.vehicle_outer_pts = pts_outer;
        m.vehicle_inner_pts = pts_inner;
    }
}

fn replenish_pedestrians(m: &WorldMap, agents: &mut [Agent]) {
    let mut p_count = agents.iter().filter(|p| p.kind == AgentKind::Pedestrian && p.on_map).count();
    while p_count < PEDESTRIAN_N {
        let agent_i = draw_off_map_agent_i(&agents, AgentKind::Pedestrian);
        if agent_i.is_none() {
            break; // no agents left to draw
        }
        let agent_i = agent_i.unwrap();

        let path_width = m.vert_sidewalk_width.max(m.horiz_sidewalk_width);
        let on_outer = calc_occupied_perim_map(agents, &m.pedestrian_outer_pts, path_width, false);
        let on_inner = calc_occupied_perim_map(agents, &m.pedestrian_inner_pts, path_width, true);

        // println!("On outer open: {:?}", on_outer.iter().filter(|&b| !b).count());
        // println!("On inner open: {:?}", on_inner.iter().filter(|&b| !b).count());

        let pose = spawn_on_two_perims(m, &m.buildings, BUILDING_PEDESTRIAN_MARGIN,
                                       (PEDESTRIAN_SIZE, PEDESTRIAN_SIZE),
                                       &m.pedestrian_outer_pts, &m.pedestrian_inner_pts,
                                       &on_outer, &on_inner, SPAWN_MARGIN);

        if let Some(pose) = pose {
            let mut agent = &mut agents[agent_i];
            // println!("Drew pose: {:?} for pedestrian\n", pose);
            agent.on_map = true;
            agent.pose = pose;
            p_count += 1;
        } else {
            // no locations are open even though we want to spawn another agent.
            // Just try again later.
            break;
        }
    }
}

fn replenish_vehicles(m: &WorldMap, agents: &mut [Agent]) {
    let mut v_count = agents.iter().filter(|p| p.kind == AgentKind::Vehicle && p.on_map).count();

    while v_count < VEHICLE_N {
        let agent_i = draw_off_map_agent_i(&agents, AgentKind::Vehicle);
        if agent_i.is_none() {
            break; // no agents left to draw
        }
        let agent_i = agent_i.unwrap();

        let path_width = m.vert_road_width.max(m.horiz_road_width) / 2;
        let on_outer = calc_occupied_perim_map(agents, &m.vehicle_outer_pts, path_width, false);
        let on_inner = calc_occupied_perim_map(agents, &m.vehicle_inner_pts, path_width, true);

        // println!("On outer: {:?}", on_outer.iter().filter(|&b| !b).count());
        // println!("On inner: {:?}", on_inner.iter().filter(|&b| !b).count());

        let pose = spawn_on_two_perims(m, &m.buildings, BUILDING_VEHICLE_MARGIN,
                                       (VEHICLE_WIDTH, VEHICLE_LENGTH),
                                       &m.vehicle_outer_pts, &m.vehicle_inner_pts,
                                       &on_outer, &on_inner, SPAWN_MARGIN);

        if let Some(pose) = pose {
            let mut agent = &mut agents[agent_i];
            // println!("Drew pose: {:?} for vehicle\n", pose);
            agent.on_map = true;
            agent.pose = pose;
            v_count += 1;
        } else {
            // no locations are open even though we want to spawn another agent.
            // Just try again later.
            break;
        }
    }
}

fn create_agents(map: &WorldMap) -> Vec<Agent> {
    let mut agents = Vec::new();
    let mut p = Agent{kind: AgentKind::Pedestrian,
                      width: PEDESTRIAN_SIZE, length: PEDESTRIAN_SIZE,
                      health: 99,
                      ..Agent::default()};
    p.folk_theory = 1.0;
    for _ in 0..(POPULATION_N/2) {
        agents.push(p.clone());
    }

    let mut v = Agent{kind: AgentKind::Vehicle,
                      width: VEHICLE_WIDTH, length: VEHICLE_LENGTH,
                      health: 99,
                      ..Agent::default()};
    v.folk_theory = 1.0;
    for _ in 0..(POPULATION_N/2) {
        agents.push(v.clone());
    }

    // add buildings
    let b = Agent{kind: AgentKind::Obstacle, on_map: true,
                  width: BUILDING_WIDTH, length: BUILDING_WIDTH,
                  ..Agent::default()};
    for building in &map.buildings {
        let mut b_agent = b.clone();
        b_agent.pose = (building.0, building.1, 0.0);
        agents.push(b_agent);
    }

    // edges of the map; top, right, bottom, left
    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: MAP_WIDTH, length: 1,
                     pose: (0, 0, 0.0),
                     ..Agent::default()};
    agents.push(edge);

    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: 1, length: MAP_HEIGHT,
                     pose: (MAP_WIDTH - 1, 0, 0.0),
                     ..Agent::default()};
    agents.push(edge);

    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: MAP_WIDTH, length: 1,
                     pose: (0, MAP_HEIGHT - 1, 0.0),
                     ..Agent::default()};
    agents.push(edge);

    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: 1, length: MAP_HEIGHT,
                     pose: (0, 0, 0.0),
                     ..Agent::default()};
    agents.push(edge);

    agents
}

fn setup_agents(map: &WorldMap, agents: &mut [Agent]) {
    replenish_pedestrians(map, agents);
    replenish_vehicles(map, agents);
}

#[derive(Debug, Clone, Rand, PartialEq)]
enum Action {
    TurnLeft,
    TurnRight,
    AccelerateForward,
    AccelerateBackward,
    Brake,
    Nothing,
    Frozen,
    ExitMap,
}

fn choose_action(_map: &WorldMap, _agents: &[Agent], agent: &Agent) -> Action {
    // very simple right now
    if !agent.on_map || agent.kind == AgentKind::Obstacle {
        return Action::Nothing;
    }
    if agent.frozen_steps > 0 {
        return Action::Frozen;
    }
    let mut rng = rand::thread_rng();
    let mut action = rng.gen();
    while action == Action::Frozen || action == Action::ExitMap {
        action = rng.gen();
    }
    action
}

fn apply_action(_map: &WorldMap, agent: &mut Agent, action: Action) {
    match action {
        Action::TurnLeft => {
            let d_theta = consts::PI / 8.0;
            let rot_pt1 = rotate_pt((agent.width as f32 / 2.0, agent.length as f32 / 2.0), -agent.pose.2);
            agent.pose.2 += d_theta;
            let rot_pt2 = rotate_pt((agent.width as f32 / 2.0, agent.length as f32 / 2.0), -agent.pose.2);
            agent.pose.0 = (agent.pose.0 as f32 + rot_pt1.0 - rot_pt2.0).round().max(1.0) as usize;
            agent.pose.1 = (agent.pose.1 as f32 + rot_pt1.1 - rot_pt2.1).round().max(1.0) as usize;
        },
        Action::TurnRight => {
            let d_theta = -consts::PI / 8.0;
            let rot_pt1 = rotate_pt((agent.width as f32 / 2.0, agent.length as f32 / 2.0), -agent.pose.2);
            agent.pose.2 += d_theta;
            let rot_pt2 = rotate_pt((agent.width as f32 / 2.0, agent.length as f32 / 2.0), -agent.pose.2);
            agent.pose.0 = (agent.pose.0 as f32 + rot_pt1.0 - rot_pt2.0).round().max(1.0) as usize;
            agent.pose.1 = (agent.pose.1 as f32 + rot_pt1.1 - rot_pt2.1).round().max(1.0) as usize;
        },
        Action::AccelerateForward => {
            agent.velocity = (agent.velocity + 1).min(3);
        },
        Action::AccelerateBackward => {
            agent.velocity = (agent.velocity - 1).max(-3);
        },
        Action::Brake => {
            if agent.velocity.abs() <= 2 {
                agent.velocity = 0;
            } else {
                agent.velocity += if agent.velocity > 0 { -2 } else { 2 };
            }
        },
        Action::Nothing => {
            return;
        },
        Action::Frozen => {
            agent.frozen_steps -= 1;
            if agent.frozen_steps == 0 {
                // agent.frozen_steps = 1;
                agent.on_map = false;
            }
            return;
        },
        Action::ExitMap => {
            agent.on_map = false;
            return;
        }
    }
    let (sin_t, cos_t) = agent.pose.2.sin_cos();

    let new_x = agent.pose.0 as f32 + agent.velocity as f32 * sin_t;
    let new_x = new_x.round().max(1.0).min(MAP_WIDTH as f32 - 1.0);
    agent.pose.0 = new_x as usize;

    let new_y = agent.pose.1 as f32 + agent.velocity as f32 * cos_t;
    let new_y = new_y.round().max(1.0).min(MAP_HEIGHT as f32 - 1.0);
    agent.pose.1 = new_y as usize;
}

fn update_agents(map: &WorldMap, agents: &mut [Agent]) {
    let mut actions = agents.iter().map(|a| choose_action(map, agents, a)).collect::<Vec<_>>();
    for (i, action) in actions.drain(..).enumerate() {
        apply_action(map, &mut agents[i], action);
    }
}

#[derive(Debug, Clone)]
struct Collision {
    agent1_i: usize,
    agent2_i: usize,
    damage1: i32,
    damage2: i32,
}

type BoundingRect = ((usize, usize), (usize, usize));

fn bounds_intersect(a: BoundingRect, b: BoundingRect) -> bool {
    let ((lx1, ly1), (hx1, hy1)) = a;
    let ((lx2, ly2), (hx2, hy2)) = b;
    if lx1 < lx2 {
        if hx1 < lx2 {
            return false;
        }
    } else {
        if hx2 < lx1 {
            return false;
        }
    }
    if ly1 < ly2 {
        if hy1 < ly2 {
            return false;
        }
    } else {
        if hy2 < ly1 {
            return false;
        }
    }
    return true;
}

fn get_velocity_vec(agent: &Agent) -> (f32, f32) {
    let (sin_t, cos_t) = agent.pose.2.sin_cos();
    (agent.velocity as f32 * sin_t, agent.velocity as f32 * cos_t)
}

fn find_collisions(agents: &[Agent]) -> Vec<Collision> {
    let mut collisions = Vec::new();

    let bounding_rects = agents.iter().map(|a|
                                        pose_size_to_bounding_rect(a.pose, (a.width, a.length)));
    let bounding_rects = bounding_rects.collect::<Vec<_>>();
    let agent_rects = agents.iter().map(|a| agent_to_rect32(a)).collect::<Vec<_>>();
    for agent1_i in 0..(agents.len() - 1) {
        let agent1 = agents[agent1_i];
        if !agent1.on_map {
            continue;
        }

        let bounds1 = bounding_rects[agent1_i];
        for agent2_i in (agent1_i + 1)..agents.len() {
            let agent2 = agents[agent2_i];
            if !agent2.on_map {
                continue;
            }
            if (agent1.frozen_steps > 0 || agent1.kind == AgentKind::Obstacle) &&
               (agent2.frozen_steps > 0 || agent2.kind == AgentKind::Obstacle) {
                // old collision
                continue;
            }
            if agent1.kind == AgentKind::Obstacle && agent2.kind == AgentKind::Obstacle {
                continue; // not important
            }

            let bounds2 = bounding_rects[agent2_i];
            // first perform lowest-res collision test
            if !bounds_intersect(bounds1, bounds2) {
                continue;
            }

            if !overlaps_rect(&agent_rects[agent1_i], &agent_rects[agent2_i]) {
                continue;
            }

            // calculate severity of the collision based on the relative angle of the agents,
            // penetration depth, relative velocity, and agent type... roughly!

            let mut intensity = 0.0;
            for &(agent_a_i, agent_b_i) in [(agent1_i, agent2_i), (agent2_i, agent1_i)].iter() {
                let agent_a = agents[agent_a_i];
                let agent_b = agents[agent_b_i];

                // project velocities and rects onto agent a's direction of motion
                let vel_a = get_velocity_vec(&agent_a);
                let vel_b = get_velocity_vec(&agent_b);

                let theta = agent_a.pose.2;

                let v_a = rotate_pt(vel_a, -theta).0;
                let v_b = rotate_pt(vel_b, -theta).0;

                let rel_vel = v_a - v_b;

                let (min_a, max_a) = project_rect(&agent_rects[agent_a_i], -theta);
                let (min_b, max_b) = project_rect(&agent_rects[agent_b_i], -theta);
                // how deeply is the front (or behind) of agent a into agent b?
                let depth = if rel_vel > 0.0 {
                    if max_a < max_b { max_a - min_b } else { 0.0 }
                } else {
                    if min_a > min_b { max_b - min_a } else { 0.0 }
                };

                intensity += depth * rel_vel.abs().max(1.0);
            }

            if agent1.kind == AgentKind::Obstacle || agent2.kind == AgentKind::Obstacle {
                intensity *= 2.0; // equal and opposite force exerted because of immovable obj.
            }

            let mut damage1 = intensity;
            let mut damage2 = intensity;
            if agent1.kind == AgentKind::Pedestrian && agent2.kind == AgentKind::Vehicle {
                damage1 *= 4.0;
            }
            if agent2.kind == AgentKind::Pedestrian && agent1.kind == AgentKind::Vehicle {
                damage2 *= 4.0;
            }
            if agent1.kind == AgentKind::Vehicle && agent2.kind == AgentKind::Vehicle {
                damage1 *= 2.0;
                damage2 *= 2.0;
            }

            let (damage1, damage2) = (damage1 as i32, damage2 as i32);
            let collision = Collision{agent1_i, agent2_i, damage1, damage2};
            println!("Collision! {:?}", collision);
            collisions.push(collision);
        }
    }

    collisions
}

fn resolve_collisions(agents: &mut [Agent], collisions: &[Collision]) {
    for collision in collisions.iter() {
        {
            let mut agent1 = &mut agents[collision.agent1_i];
            if agent1.kind != AgentKind::Obstacle && agent1.frozen_steps == 0 {
                agent1.health = (agent1.health - collision.damage1).max(0);
                agent1.frozen_steps = FROZEN_STEPS;
                agent1.velocity = 0;
            }
        }
        let mut agent2 = &mut agents[collision.agent2_i];
        if agent2.kind != AgentKind::Obstacle && agent2.frozen_steps == 0 {
            agent2.health = (agent2.health - collision.damage2).max(0);
            agent2.frozen_steps = FROZEN_STEPS;
            agent2.velocity = 0;
        }
    }
}

fn run_timestep(map: &WorldMap, agents: &mut [Agent]) {
    update_agents(map, agents);
    let collisions = find_collisions(agents);
    resolve_collisions(agents, &collisions);

    replenish_pedestrians(map, agents);
    replenish_vehicles(map, agents);
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
    let mut map = create_map();
    setup_map_paths(&mut map);

    let mut agents = create_agents(&map);
    setup_agents(&map, &mut agents);
    let state = WorldState {map, agents: Arc::new(Mutex::new(agents))};

    let thread_state = state.clone();
    thread::spawn(move || {
        loop {
            thread::sleep(std::time::Duration::from_millis(100));
            run_timestep(&thread_state.map, &mut *thread_state.agents.lock().unwrap());
        }
    });

    rocket::ignite()
        .manage(state.clone())
        .mount("/", routes![index, get_agents, resources])
        .attach(Template::fairing())
        .launch();
}

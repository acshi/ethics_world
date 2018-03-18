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

use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::f32::{self, consts};
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

const MAP_WIDTH: usize = 70;
const MAP_HEIGHT: usize = 70;
const MAP_SIZE: usize = MAP_WIDTH * MAP_HEIGHT;
const BUILDING_N: usize = 30;
const CROSSWALK_N: usize = 6;
const EXTRA_OBSTACLE_N: usize = 6;
const POPULATION_N: usize = 24; // one-half this for each of pedestrians and vehicles
const PEDESTRIAN_N: usize = 6; // those on map at one time
const VEHICLE_N: usize = 6;
const SPAWN_MARGIN: usize = 5; // clearance from other agents when spawning

const BUILDING_WIDTH: usize = 3; // just a facade
const BUILDING_SPACING: usize = 12;
const CROSSWALK_WIDTH: usize = 2;
const CROSSWALK_SPACING: usize = 6;
const PEDESTRIAN_SIZE: usize = 1;
const VEHICLE_WIDTH: usize = 4;
const VEHICLE_LENGTH: usize = 11;
const SIDEWALK_MIN_WIDTH: usize = 3;
const SIDEWALK_MAX_WIDTH: usize = 6;
const LANE_MIN_WIDTH: usize = 6;
const LANE_MAX_WIDTH: usize = 10;

// distance at which the pedestrian is considered adjacent to the building
const BUILDING_PEDESTRIAN_MARGIN: usize = 3;
// distance (besides crosswalk) at which the vehicle is considered adjacent to the building
const BUILDING_VEHICLE_MARGIN: usize = 5;

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

fn create_buildings(m: &mut WorldMap) {
    let mut rng = rand::thread_rng();

    let outer_perim = m.horiz_road_length * 2 + m.vert_road_length * 2;
    let inner_perim = outer_perim - (m.vert_road_width * 2 + m.horiz_road_width * 2 +
                                     m.vert_sidewalk_width * 2 + m.horiz_sidewalk_width * 2 +
                                     (BUILDING_WIDTH - 1) * 2
                                    ) * 2;
    let total_perim = outer_perim + inner_perim;
    let mut chosen_perims = [0usize; BUILDING_N];
    for i in 0..BUILDING_N {
        let mut perim_loc;
        let attempts = 100;
        let mut attempt_i = 0;
        loop {
            perim_loc = rng.gen_range(0, total_perim);
            // check that existing buildings are far enough away
            // only compare with those on the correct inner/outer path
            if perim_loc < outer_perim {
                if chosen_perims.iter().take(i).all(|&p|
                        p >= outer_perim ||
                        mod_dist(p, perim_loc, outer_perim) > BUILDING_SPACING) {
                    break;
                }
            } else {
                let perim_loc = perim_loc - outer_perim;
                if chosen_perims.iter().take(i).all(|&p|
                        p < outer_perim ||
                        mod_dist(p - outer_perim, perim_loc, inner_perim) > BUILDING_SPACING) {
                    break;
                }
            }

            attempt_i += 1;
            if attempt_i > attempts {
                return; // probably no solution is left for current choices.
            }
        }
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
        let attempts = 100;
        let mut attempts_i = 0;
        loop {
            perim_loc = rng.gen_range(0, crosswalk_perim);
            // check that existing crosswalks are far enough away
            if chosen_crosswalks.iter().take(i).all(|p| (*p as i32 - perim_loc as i32).abs() > CROSSWALK_SPACING as i32) {
                break;
            }
            attempts_i += 1;
            if attempts_i > attempts {
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

#[derive(Clone, Copy, Debug, Default)]
struct Agent {
    on_map: bool,
    pose: (usize, usize, f32), // x, y, theta (rads)
    width: usize, // perpendicular to theta
    length: usize, // in direction of theta
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
        let mut state = serializer.serialize_struct("Agent", 6)?;
        state.serialize_field("on_map", &self.on_map)?;
        state.serialize_field("x", &self.pose.0)?;
        state.serialize_field("y", &self.pose.1)?;
        state.serialize_field("theta", &self.pose.2)?;
        state.serialize_field("width", &self.width)?;
        state.serialize_field("length", &self.length)?;
        state.end()
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

// manhattan distance/1 norm, considers that the positions are upper-left corners
// and that the objects have their sizes expanding to the right and down.
fn obj_dist_1<T>(a: (T, T), size_a: (T, T), b: (T, T), size_b: (T, T)) -> T
    where T: std::ops::Add<Output=T> + std::ops::Sub<Output=T> + PartialOrd
{
    let ((x1, y1), (x2, y2)) = (a, b);
    let ((sx1, sy1), (sx2, sy2)) = (size_a, size_b);
    return if x1 > x2 { difference(x1, x2 + sx2) } else { difference(x2, x1 + sx1) } +
           if y1 > y2 { difference(y1, y2 + sy2) } else { difference(y2, y1 + sy1) };
}

fn calc_total_perim(pts: &RectUsize) -> usize {
    let lines = points_to_lines_usize(pts);
    lines.into_iter().map(|a| dist_1(a)).sum()
}

fn perim_i_to_pt(perim_i: usize, perims: &Vec<usize>, lines: &Vec<LineUsize>) -> (usize, usize) {
    let mut perim_val = perim_i;
    for i in 0..lines.len() {
        let line = lines[i];
        let perim = perims[i];
        if perim_val < perim {
            let ((x1, y1), (x2, y2)) = line;
            let sx = (x2 as i32 - x1 as i32).signum();
            let sy = (y2 as i32 - y1 as i32).signum();
            let pt = ((x1 as i32 + sx * perim_val as i32) as usize,
                      (y1 as i32 + sy * perim_val as i32) as usize);
            // include the boundary such that results look like they are from [min, max)
            // even when we actually iterate from high to low
            let pt = (pt.0 - if sx == -1 { 1 } else { 0 },
                      pt.1 - if sy == -1 { 1 } else { 0 });
            return pt;
        }
        perim_val -= perim;
    }
    panic!("Perim_i {} is invalid, exceeds perimeter total {}", perim_i, perims.iter().sum::<usize>());
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
        let perim_choices = perim_choices.filter(|&i|
                            obj_dist_1(building, (BUILDING_WIDTH, BUILDING_WIDTH),
                                   perim_i_to_pt(i, &perims, &lines), size) <= building_margin);
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

    let pt = perim_i_to_pt(perim_loc, &perims, &lines);

    // calculate pose from the line that we are on
    let mut perim_val = perim_loc;
    let mut line_i = 4;
    for (i, &perim) in perims.iter().enumerate() {
        if perim_val < perim {
            line_i = i;
            break;
        }
        perim_val -= perim;
    }

    let ((x1, y1), (x2, y2)) = lines[line_i];
    let pose;
    if is_inner {
        if line_i == 0 {
            if pt.0 < x1 + size.1 - m.vert_road_width / 2 {
                return None;
            }
            pose = (pt.0 + 1, pt.1 - size.0, -consts::PI / 2.0);
        } else if line_i == 1 {
            if pt.1 < y1 + size.1 - m.horiz_road_width / 2 {
                return None;
            }
            pose = (pt.0 + size.0, pt.1 + 1, consts::PI);
        } else if line_i == 2 {
            if pt.0 > x1 - size.1 + m.vert_road_width / 2 {
                return None;
            }
            pose = (pt.0, pt.1 + size.0, consts::PI / 2.0);
        } else if line_i == 3 {
            if pt.1 > y1 - size.1 + m.horiz_road_width / 2 {
                return None;
            }
            pose = (pt.0 - size.0, pt.1, 0.0);
        } else {
            panic!("Should not be possible");
        }
    } else {
        if line_i == 0 {
            pose = (pt.0.min(x2 - size.0), pt.1 + size.0, consts::PI / 2.0);
        } else if line_i == 1 {
            pose = (pt.0 - size.0, pt.1.min(y2 - size.0), 0.0);
        } else if line_i == 2 {
            pose = (pt.0.max(x2 + size.0), pt.1 - size.0, -consts::PI / 2.0);
        } else if line_i == 3 {
            pose = (pt.0 + size.0, pt.1.max(y2 + size.0), consts::PI);
        } else {
            panic!("Should not be possible");
        }
    }

    println!("Chose perim_loc: {} at {:?} with pose {:?}",
             perim_loc, pt, pose);

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

    let attempts = 200;
    let mut res: Option<(usize, usize, f32)> = None;
    let mut i = 0;
    while i < attempts && res.is_none() {
        let building = *rng.choose(buildings)?;

        // res = spawn_from_perim(m, building, building_margin, size,
        //                        pts_a, occupied_a, margin, false);
        if res.is_none() {
            res = spawn_from_perim(m, building, building_margin, size,
                                   pts_b, occupied_b, margin, true);
        }
        i += 1;
    }
    res
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
    let projected = rect.iter().map(|p| rotate_pt(*p, angle).0).collect::<Vec<_>>();
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
        let (min1, max1) = project_rect(q1, normal);
        let (min2, max2) = project_rect(q2, normal);
        // include epsilon value for rounding/imprecision error.
        // so that adjacent things are not considered overlapping
        if max1 <= (min2 + 1e-4) || max2 <= (min1 + 1e-4) {
            return false;
        }
    }

    true
}

// if there is an overlap returns the approximate perimeter location of the overlap.
// as elsewhere, this goes through the perimeter clockwise top, right, bottom, left
fn overlaps_path(agent: &Agent, path: &RectUsize, path_width: usize, is_inner: bool) -> Option<usize> {
    let path_width = path_width as f32;

    let (width, length) = (agent.width as f32, agent.length as f32);
    let (x, y, theta) = (agent.pose.0 as f32, agent.pose.1 as f32, agent.pose.2);
    let rot_width = rotate_pt((width, 0.0), -theta);
    let rot_length = rotate_pt((0.0, length), -theta);
    let agent_rect = [(x, y),
                      (x + rot_width.0, y + rot_width.1),
                      (x + rot_width.0 + rot_length.0, y + rot_width.1 + rot_length.1),
                      (x + rot_length.0, y + rot_length.1)];

    let path_lines = points_usize_to_lines_f32(path);
    println!("len: {}", path_lines.len());
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
        println!("Path line rect {}: {:?}", i, line_rect);
        if overlaps_rect(&agent_rect, &line_rect) {
            println!("Overlaps");
            let perim_pos = if x1_eq_x2 { (y - y1).abs() } else { (x - x1).abs() };
            return Some(perim_loc + perim_pos as usize);
        }
        perim_loc += dist_1(line) as usize;
    }

    None
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
fn calc_occupied_perim_map(agents: &WorldAgents, path: &RectUsize, path_width: usize, is_inner: bool) -> Vec<bool>
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

    for agent in agents.pedestrians.iter().chain(agents.vehicles.iter()) {
        if !agent.on_map {
            continue;
        }

        let overlap_loc = overlaps_path(agent, path, path_width as usize, is_inner);
        if overlap_loc.is_none() {
            continue;
        }
        let overlap_loc = overlap_loc.unwrap();

        let (width, length) = (agent.width as f32, agent.length as f32);
        let (x, y, theta) = (agent.pose.0 as f32, agent.pose.1 as f32, agent.pose.2);
        let rot_width = rotate_pt((width, 0.0), -theta);
        let rot_length = rotate_pt((0.0, length), -theta);
        let agent_rect = [(x, y),
                          (x + rot_width.0, y + rot_width.1),
                          (x + rot_width.0 + rot_length.0, y + rot_width.1 + rot_length.1),
                          (x + rot_length.0, y + rot_length.1)];

        // println!("Examining agent with rect: {:?}", agent_rect);

        // we only need to check along the path as far as this maximum dimension squared
        // from the overlap location calculated above.
        let agent_max_dim_sq = agent.width.pow(2) + agent.length.pow(2);

        for (perim_loc, cell_rect) in path_cell_rects.iter().enumerate() {
            if occupied_perims[perim_loc] {
                continue;
            }
            let overlap_loc_dist_sq = mod_dist(perim_loc, overlap_loc, perim_total).pow(2);
            if overlap_loc_dist_sq > agent_max_dim_sq {
                continue;
            }

            if overlaps_rect(&agent_rect, &cell_rect) {
                occupied_perims[perim_loc] = true;
            //     println!("Overlaps perim_loc w/ cell rect {:?}", cell_rect);
            // } else {
            //     println!("Does not overlap perim_loc w/ cell rect {:?}", cell_rect);
            }
        }
    }
    occupied_perims
}

fn draw_off_map_agent(agents: &mut [Agent]) -> &mut Agent {
    let mut rng = rand::thread_rng();
    let off_map_agents = agents.iter()
                               .enumerate()
                               .filter_map(|(i, a)| if !a.on_map { Some(i) } else { None })
                               .collect::<Vec<_>>();
    let i = off_map_agents[rng.gen_range(0, off_map_agents.len())];
    &mut agents[i]
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

fn replenish_pedestrians(m: &WorldMap, agents: &mut WorldAgents) {
    let mut p_count = agents.pedestrians.iter().filter(|p| p.on_map).count();
    while p_count < PEDESTRIAN_N {
        let path_width = m.vert_sidewalk_width.max(m.horiz_sidewalk_width);
        let on_outer = calc_occupied_perim_map(agents, &m.pedestrian_outer_pts, path_width, false);
        let on_inner = calc_occupied_perim_map(agents, &m.pedestrian_inner_pts, path_width, true);

        println!("On outer open: {:?}", on_outer.iter().filter(|&b| !b).count());
        println!("On inner open: {:?}", on_inner.iter().filter(|&b| !b).count());

        let pose = spawn_on_two_perims(m, &m.buildings, BUILDING_PEDESTRIAN_MARGIN,
                                       (PEDESTRIAN_SIZE, PEDESTRIAN_SIZE),
                                       &m.pedestrian_outer_pts, &m.pedestrian_inner_pts,
                                       &on_outer, &on_inner, SPAWN_MARGIN);

        if let Some(pose) = pose {
            let mut agent = draw_off_map_agent(&mut agents.pedestrians);
            println!("Drew pose: {:?} and pedestrian: {:?}", pose, agent);
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

fn replenish_vehicles(m: &WorldMap, agents: &mut WorldAgents) {
    let mut v_count = agents.vehicles.iter().filter(|p| p.on_map).count();

    while v_count < VEHICLE_N {
        let path_width = m.vert_road_width.max(m.horiz_road_width);
        let on_outer = calc_occupied_perim_map(agents, &m.vehicle_outer_pts, path_width, false);
        let on_inner = calc_occupied_perim_map(agents, &m.vehicle_inner_pts, path_width, true);

        println!("On outer: {:?}", on_outer.iter().filter(|&b| !b).count());
        println!("On inner: {:?}", on_inner.iter().filter(|&b| !b).count());

        let pose = spawn_on_two_perims(m, &m.buildings, BUILDING_VEHICLE_MARGIN,
                                       (VEHICLE_WIDTH, VEHICLE_LENGTH),
                                       &m.vehicle_outer_pts, &m.vehicle_inner_pts,
                                       &on_outer, &on_inner, SPAWN_MARGIN);

        if let Some(pose) = pose {
            let mut agent = draw_off_map_agent(&mut agents.vehicles);
            println!("Drew pose: {:?} and vehicle: {:?}", pose, agent);
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


fn create_agents(map: &WorldMap) -> WorldAgents {
    let p = Agent{width: PEDESTRIAN_SIZE, length: PEDESTRIAN_SIZE, ..Agent::default()};
    let v = Agent{width: VEHICLE_WIDTH, length: VEHICLE_LENGTH, ..Agent::default()};
    let pedestrians = [p; POPULATION_N/2];
    let vehicles = [v; POPULATION_N/2];

    let agents = WorldAgents { pedestrians: Box::new(pedestrians),
                               vehicles: Box::new(vehicles) };
    agents
}

fn setup_agents(map: &WorldMap, agents: &mut WorldAgents) {
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

    rocket::ignite()
        .manage(state.clone())
        .mount("/", routes![index, get_agents, resources])
        .attach(Template::fairing())
        .launch();
}

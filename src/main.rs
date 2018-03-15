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
use std::f32;
use rocket::{State};
use rocket::response::{NamedFile};
use rocket_contrib::{Json, Value};
use rand::Rng;
use serde::ser::SerializeSeq;
use serde::ser::SerializeStruct;

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
const POPULATION_N: usize = 24; // one-half this for each of pedestrians and vehicles
const PEDESTRIAN_N: usize = 6; // those on map at one time
const VEHICLE_N: usize = 6;
const SPAWN_MARGIN: usize = 5;

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

#[derive(Clone)]
struct WorldMap {
    grid: Box<[Cell]>,
    buildings: [(usize, usize); BUILDING_N],
    grid_width: usize,
    horiz_road_width: usize,
    vert_road_width: usize,
    horiz_sidewalk_width: usize,
    vert_sidewalk_width: usize,
    horiz_road_length: usize,
    vert_road_length: usize,
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

    WorldMap{grid: Box::new(grid), buildings, grid_width: MAP_WIDTH,
             horiz_road_width, vert_road_width,
             horiz_sidewalk_width, vert_sidewalk_width,
             horiz_road_length, vert_road_length}
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

type LineUsize = ((usize, usize), (usize, usize));
type LineF32 = ((f32, f32), (f32, f32));

fn points_to_lines_f32(pts: &[(f32, f32); 4]) -> Vec<LineF32> {
    pts.iter().cloned().zip(pts.iter().skip(1).chain(pts.iter().take(1)).cloned()).collect()
}

fn points_to_lines_usize(pts: &[(usize, usize); 4]) -> Vec<LineUsize> {
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

// makes the assumption that points form a strict rectangle (only y or x change at a time)
fn draw_from_perim(pts: &[(usize, usize); 4], existing: &mut Vec<usize>, margin: usize) -> Option<(usize, usize)> {
    let mut rng = rand::thread_rng();

    let lines = points_to_lines_usize(pts);

    let perims = lines.iter().map(|&a| dist_1(a));
    let perims = perims.collect::<Vec<_>>();
    let perim_total = perims.iter().sum();

    let attempts = 100;
    let mut perim_loc = 0;
    for i in 0..attempts {
        perim_loc = rng.gen_range(0, perim_total);
        // check that existing buildings are far enough away
        if existing.iter().all(|p| (*p as i32 - perim_loc as i32).abs() > margin as i32) {
            break;
        }
        if i >= attempts - 1 {
            return None;
        }
    }
    existing.push(perim_loc);

    let mut perim_val = perim_loc;
    for i in 0..lines.len() {
        let line = lines[i];
        let perim = perims[i];
        if perim_val < perim {
            let ((x1, y1), (x2, y2)) = line;
            let sx = (x2 as i32 - x1 as i32).signum();
            let sy = (y2 as i32 - y1 as i32).signum();
            return Some(((x1 as i32 + sx * perim as i32) as usize,
                         (y1 as i32 + sy * perim as i32) as usize));
        }
        perim_val -= perim;
    }

    None
}

fn calc_total_perim(pts: &[(usize, usize); 4]) -> usize {
    let lines = points_to_lines_usize(pts);
    lines.into_iter().map(|a| dist_1(a)).sum()
}

fn draw_from_two_perims(pts_a: &[(usize, usize); 4], pts_b: &[(usize, usize); 4],
                        existing_a: &mut Vec<usize>, existing_b: &mut Vec<usize>,
                        margin: usize) -> Option<(usize, usize)> {
    let mut rng = rand::thread_rng();

    let perim_a = calc_total_perim(pts_a);
    let perim_b = calc_total_perim(pts_b);
    let perim_total = perim_a + perim_b;
    let perim_loc = rng.gen_range(0, perim_total);
    let mut res;
    if perim_loc < perim_a {
        res = draw_from_perim(pts_a, existing_a, margin);
    } else {
        res = draw_from_perim(pts_b, existing_b, margin);
    }
    if res.is_none() {
        if perim_loc < perim_a {
            res = draw_from_perim(pts_b, existing_b, margin);
        } else {
            res = draw_from_perim(pts_a, existing_a, margin);
        }
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
fn project_rect(rect: &[(f32, f32); 4], angle: f32) -> (f32, f32) {
    let projected = rect.iter().map(|p| rotate_pt(*p, angle).0).collect::<Vec<_>>();
    (projected.iter().fold(f32::MAX, |a, &b| a.min(b)), projected.iter().fold(f32::MIN, |a, &b| a.max(b)))
}

fn overlaps_rect(q1: &[(f32, f32); 4], q2: &[(f32, f32); 4]) -> bool {
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
        if max1 < min2 || max2 < min1 {
            return false;
        }
    }

    true
}

// if there is an overlap returns the approximate perimeter location of the overlap
// as elsewhere, this goes through the perimeter clockwise top, right, bottom, left
fn overlaps_path(agent: &Agent, path: &[(usize, usize); 4], path_width: usize) -> Option<usize> {
    let (width, length) = (agent.width as f32, agent.length as f32);
    let (x, y, theta) = (agent.pose.0 as f32, agent.pose.1 as f32, agent.pose.2);
    let rot_width = rotate_pt((width, 0.0), theta);
    let rot_length = rotate_pt((0.0, length), theta);
    let agent_rect = [(x, y),
                      (x + rot_width.0, y + rot_width.1),
                      (x + rot_width.0 + rot_length.0, y + rot_width.1 + rot_length.1),
                      (x + rot_length.0, y + rot_length.1)];
    let path_lines = points_usize_to_lines_f32(path);
    let mut perim_loc = 0;
    for line in path_lines {
        let ((x1, y1), (x2, y2)) = line;
        let x_width = if x1 == x2 { path_width as f32 } else { 0.0 };
        let y_width = if y1 == y2 { path_width as f32 } else { 0.0 };
        let line_rect = [(x1, y1),
                         (x1 + x_width, y1 + y_width),
                         (x2 + x_width, y2 + y_width),
                         (x2, y2)];
        if overlaps_rect(&agent_rect, &line_rect) {
            let perim_pos = if x1 == x2 { (y - y1).abs() } else { (x - x1).abs() };
            return Some(perim_loc + perim_pos as usize);
        }
        perim_loc += dist_1(line) as usize;
    }

    None
}

fn find_occupied_perims(agents: &WorldAgents, path: &[(usize, usize); 4], path_width: usize) -> Vec<usize> {
    let perims1 = agents.pedestrians.iter().filter_map(|a| overlaps_path(a, path, path_width));
    let perims2 = agents.vehicles.iter().filter_map(|a| overlaps_path(a, path, path_width));
    perims1.chain(perims2).collect()
}

fn replenish_agents(m: &WorldMap, agents: &mut WorldAgents) {
    loop {
        let p_count = agents.pedestrians.iter().filter(|p| p.on_map).count();
        if p_count >= PEDESTRIAN_N {
            break;
        }

        let right_outer = BUILDING_WIDTH + m.vert_sidewalk_width + m.horiz_road_length;
        let bottom_outer = BUILDING_WIDTH + m.horiz_sidewalk_width + m.vert_road_length;
        let pts_outer = [(BUILDING_WIDTH, BUILDING_WIDTH),
                         (right_outer, BUILDING_WIDTH),
                         (right_outer, bottom_outer),
                         (BUILDING_WIDTH, bottom_outer)];

        let left_inner = BUILDING_WIDTH + m.vert_sidewalk_width + m.vert_road_width;
        let top_inner = BUILDING_WIDTH + m.horiz_sidewalk_width + m.horiz_road_width;
        let right_inner = right_outer - m.vert_road_width - m.vert_sidewalk_width;
        let bottom_inner = bottom_outer - m.horiz_road_width - m.horiz_sidewalk_width;
        let pts_inner = [(left_inner, top_inner),
                         (right_inner, top_inner),
                         (right_inner, bottom_inner),
                         (left_inner, bottom_inner)];

        let mut on_outer = find_occupied_perims(agents, &pts_outer, SPAWN_MARGIN);
        let mut on_inner = find_occupied_perims(agents, &pts_inner, SPAWN_MARGIN);

        draw_from_two_perims(&pts_outer, &pts_inner, &mut on_outer, &mut on_inner, SPAWN_MARGIN);
    }
    // let v_count = agents.vehicles.iter().filter(|p| p.on_map).count();
}

fn create_agents(map: &WorldMap) -> WorldAgents {
    let mut pedestrians = [Agent::default(); POPULATION_N/2];
    let mut vehicles = [Agent::default(); POPULATION_N/2];

    let mut agents = WorldAgents { pedestrians: Box::new(pedestrians), vehicles: Box::new(vehicles) };
    replenish_agents(map, &mut agents);
    agents
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
    let agents = create_agents(&map);
    let state = WorldState {map, agents: Arc::new(Mutex::new(agents))};

    rocket::ignite()
        .manage(state.clone())
        .mount("/", routes![index, get_agents, resources])
        .attach(Template::fairing())
        .launch();
}

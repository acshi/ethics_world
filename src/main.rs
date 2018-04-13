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
#[macro_use] extern crate lazy_static;
extern crate fnv;
extern crate config;

use std::fmt;
use std::path::{Path, PathBuf};
use std::cell::UnsafeCell;
use std::rc::Rc;
use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};
use std::sync::mpsc::channel;
use std::f32::{self, consts};
use std::thread;
use std::io::{self, Write};
use std::fs;
use fnv::FnvHashMap;
use rocket::{State};
use rocket::response::{NamedFile};
use rocket_contrib::{Json, Value};
use rand::{Rng, XorShiftRng};
use serde::ser::{SerializeStruct, SerializeTuple};

mod general_search;
#[allow(unused_imports)]
use general_search::{PriorityQueue, SearchQueue, SearchProblemTrait, SearchNode,
                     create_search_problem, tree_search};

#[cfg(test)]
mod tests;

const MAP_WIDTH: u32 = 74;
const MAP_HEIGHT: u32 = 74;
const MAP_SIZE: u32 = MAP_WIDTH * MAP_HEIGHT;
const THETA_PER_INC: f32 = consts::PI / 8.0;
const TWO_PI_INCS: u32 = ((2.0 * consts::PI) / (THETA_PER_INC) + 0.5) as u32;
const PI_INCS: u32 = TWO_PI_INCS / 2;
const PI_OVER_TWO_INCS: u32 = PI_INCS / 2;
const BUILDING_N: usize = 2;//4;
const CROSSWALK_N: usize = 6;
// const EXTRA_OBSTACLE_N: u32 = 6;
const SPAWN_MARGIN: u32 = 3; // clearance from other agents when spawning

const BUILDING_WIDTH: u32 = 3; // just a facade
const BUILDING_SPACING: u32 = 12;
const CROSSWALK_WIDTH: u32 = 2;
const CROSSWALK_SPACING: u32 = 6;
const PEDESTRIAN_SIZE: u32 = 1;
const VEHICLE_WIDTH: u32 = 4;
const VEHICLE_LENGTH: u32 = 10;
const SIDEWALK_MIN_WIDTH: u32 = 3;
const SIDEWALK_MAX_WIDTH: u32 = 6;
const LANE_MIN_WIDTH: u32 = 7;
const LANE_MAX_WIDTH: u32 = 11;

const FROZEN_STEPS: u32 = 6;
const ATTEMPTS: u32 = 100;

// distance at which the pedestrian is considered adjacent to the building
const BUILDING_PEDESTRIAN_MARGIN: u32 = 3;
// distance (besides crosswalk) at which the vehicle is considered adjacent to the building
const BUILDING_VEHICLE_MARGIN: u32 = 6;

#[derive(Debug, Deserialize)]
struct ConfSettings {
	population_n: usize, // one-half this for each of pedestrians and vehicles
    pedestrian_n: usize, // those on map at one time
    vehicle_n: usize,
    total_health: u32,
    heal_reward: i32,

    forward_accel: u32,
    backward_accel: u32,
    max_forward_vel: u32,
    max_backward_vel: u32,
    brake_power: u32,

    timestep_limit: u32,
    reset_counters_at_timestep: u32,

    // for single agent type testing
    use_single_agent_type: bool,
    habit_theory: f32,
    folk_theory: f32,
    internalization_theory: f32,
    choice_restriction_theory: bool,

    // internalization theory search process
    off_path_penalty: f32,
    off_lane_penalty: f32,
    damage_penalty: f32,
    moral_forgiveness: f32,

    // for astar action search process "folk theory"
    astar_depth: u32,
    search_trip_complete_reward: f32,

    // for the MDP/Q-Learning processses "habit theory"
    state_avoid_dist: u32,
    explore_one_in_max: u32,
    explore_consecutive_limit: u32,
    full_explore_until: u32,
    q_learn_rate: f32,
    q_discount: f32,
    trip_reward: f32,
    damage_coef: f32,
    anneal_factor: f32,
    anneal_start: u32,
    anneal_end: u32,
    debug_choices_after: u32,

    fast_steps_per_update: u32,
    slow_step_ms: u32,
    fast_steps_update_q_empties: u32,
}

lazy_static! {
    static ref C: ConfSettings = {
        let mut conf = config::Config::default();
    	match conf.merge(config::File::with_name("settings")) {
    		Ok(_) => (),
    		Err(e) => {
    			eprintln!("Error opening settings.toml file: {}", e);
    			std::process::exit(-1);
    		}
    	}

        let settings: ConfSettings = match conf.try_into() {
    		Ok(settings) => settings,
    		Err(e) => {
    			eprintln!("Configuration error in settings.toml file: {}", e);
    			std::process::exit(-1);
    		}
    	};
        settings
    };
}

thread_local! {
    static RAND: Rc<UnsafeCell<XorShiftRng>> = {
        Rc::new(UnsafeCell::new(XorShiftRng::new_unseeded()))
    };
}

// basically just the same as https://doc.rust-lang.org/rand/src/rand/thread_rng.rs.html#97-99
#[derive(Clone, Debug)]
pub struct RepeatableRng {
    rng: Rc<UnsafeCell<XorShiftRng>>
}

impl Rng for RepeatableRng {
    #[inline(always)]
    fn next_u32(&mut self) -> u32 {
        unsafe { (*self.rng.get()).next_u32() }
    }
    #[inline(always)]
    fn next_u64(&mut self) -> u64 {
        unsafe { (*self.rng.get()).next_u64() }
    }
    fn fill_bytes(&mut self, bytes: &mut [u8]) {
        unsafe { (*self.rng.get()).fill_bytes(bytes) }
    }
}

fn repeatable_rand() -> RepeatableRng {
    RepeatableRng { rng: RAND.with(|r| r.clone()) }
}

bitflags! {
    struct Cell: u8 {
        const OUTER_LANE =  0b00000001;
        const INNER_LANE =  0b00000010;
        const OBSTACLE =    0b00000100;
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

#[derive(Clone, Copy)]
struct Building {
    x: u32,
    y: u32,
    agent_i: usize,
}

impl serde::Serialize for Building {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where S: serde::Serializer
    {
        let mut state = serializer.serialize_tuple(2)?;
        state.serialize_element(&self.x)?;
        state.serialize_element(&self.y)?;
        state.end()
    }
}

#[derive(Clone)]
struct WorldMap {
    grid: Box<[Cell]>,
    buildings: Vec<Building>,
    grid_width: u32,
    horiz_road_width: u32,
    vert_road_width: u32,
    horiz_sidewalk_width: u32,
    vert_sidewalk_width: u32,
    horiz_road_length: u32,
    vert_road_length: u32,

    pedestrian_outer_pts: [(u32, u32); 4],
    pedestrian_inner_pts: [(u32, u32); 4],
    vehicle_outer_pts: [(u32, u32); 4],
    vehicle_inner_pts: [(u32, u32); 4],
    vehicle_inner_rects: [[(f32, f32); 4]; 4],
    vehicle_outer_rects: [[(f32, f32); 4]; 4],
    corners: [BoundingRectI32; 4],
}

impl serde::Serialize for WorldMap {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where S: serde::Serializer
    {
        let mut state = serializer.serialize_struct("WorldMap", 3)?;
        state.serialize_field("grid", &self.grid)?;
        state.serialize_field("buildings", &self.buildings)?;
        state.serialize_field("grid_width", &self.grid_width)?;
        state.end()
    }
}

fn fill_map_rect(grid: &mut [Cell], left: u32, top: u32,
                                    width: u32, height: u32, mask: Cell) {
    // println!("left: {} top: {} width: {} height: {}", left, top, width, height);
    for y in top..(top + height) {
        let mut grid_i = y * MAP_WIDTH + left;
        for _ in 0..width {
            grid[grid_i as usize] |= mask;
            grid_i += 1;
        }
    }
}

fn create_buildings(m: &mut WorldMap) {
    if BUILDING_N <= 1 {
        panic!("Building number too low! must be at least 2 to create destinations.");
    }

    let mut rng = repeatable_rand(); //rand::thread_rng();

    let outer_perim = m.horiz_road_length * 2 + m.vert_road_length * 2;
    let inner_perim = outer_perim - (m.vert_road_width * 2 + m.horiz_road_width * 2 +
                                     m.vert_sidewalk_width * 2 + m.horiz_sidewalk_width * 2 +
                                     (BUILDING_WIDTH - 1) * 2
                                    ) * 2;
    let total_perim = outer_perim + inner_perim;
    let mut chosen_perims = [0u32; BUILDING_N];

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
                fill_map_rect(&mut m.grid[..], left + perim_loc, 0, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
                m.buildings.push(Building{ x: left + perim_loc, y: 0, agent_i: 0 });
                continue;
            }
            let perim_loc = perim_loc - m.horiz_road_length;
            if perim_loc < m.vert_road_length {
                fill_map_rect(&mut m.grid[..], right, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
                m.buildings.push(Building{ x: right, y: top + perim_loc, agent_i: 0});
                continue;
            }
            let perim_loc = perim_loc - m.vert_road_length;
            if perim_loc < m.horiz_road_length {
                fill_map_rect(&mut m.grid[..], left + perim_loc, bottom, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
                m.buildings.push(Building{ x: left + perim_loc, y: bottom, agent_i: 0});
                continue;
            }
            let perim_loc = perim_loc - m.horiz_road_length;
            fill_map_rect(&mut m.grid[..], 0, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
            m.buildings.push(Building{ x: 0, y: top + perim_loc, agent_i: 0});
        } else {
            let perim_loc = perim_loc - outer_perim;
            let inner_h_length = m.horiz_road_length - 2 * m.vert_road_width - 2 * m.vert_sidewalk_width - (BUILDING_WIDTH - 1);
            let inner_v_length = m.vert_road_length - 2 * m.horiz_road_width - 2 * m.horiz_sidewalk_width - (BUILDING_WIDTH - 1);
            let left = BUILDING_WIDTH + 2 * m.vert_sidewalk_width + m.vert_road_width;
            let top = BUILDING_WIDTH + 2 * m.horiz_sidewalk_width + m.horiz_road_width;
            let right = m.horiz_road_length - m.vert_road_width;
            let bottom = m.vert_road_length - m.horiz_road_width;
            if perim_loc < inner_h_length {
                fill_map_rect(&mut m.grid[..], left + perim_loc, top, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
                m.buildings.push(Building{ x: left + perim_loc, y: top, agent_i: 0});
                continue;
            }
            let perim_loc = perim_loc - inner_h_length;
            if perim_loc < inner_v_length {
                fill_map_rect(&mut m.grid[..], right, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
                m.buildings.push(Building{ x: right, y: top + perim_loc, agent_i: 0});
                continue;
            }
            let perim_loc = perim_loc - inner_v_length;
            if perim_loc < inner_h_length {
                fill_map_rect(&mut m.grid[..], left + perim_loc, bottom, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
                m.buildings.push(Building{ x: left + perim_loc, y: bottom, agent_i: 0});
                continue;
            }
            let perim_loc = perim_loc - inner_h_length;
            fill_map_rect(&mut m.grid[..], left, top + perim_loc, BUILDING_WIDTH, BUILDING_WIDTH, Cell::OBSTACLE);
            m.buildings.push(Building{ x: left, y: top + perim_loc, agent_i: 0});
        }
    }
}

fn create_crosswalks(m: &mut WorldMap) {
    let mut rng = repeatable_rand(); //rand::thread_rng();

    let horiz_crosswalk_perim = m.horiz_road_length - 2 * m.vert_road_width - CROSSWALK_WIDTH;
    let vert_crosswalk_perim = m.vert_road_length - 2 * m.horiz_road_width - CROSSWALK_WIDTH;
    let crosswalk_perim = horiz_crosswalk_perim * 2 + vert_crosswalk_perim * 2;
    let mut chosen_crosswalks = [0u32; CROSSWALK_N];
    for i in 0..CROSSWALK_N {
        let mut perim_loc;
        let mut attempts_i = 0;
        loop {
            perim_loc = rng.gen_range(0, crosswalk_perim);
            // check that existing crosswalks are far enough away
            if chosen_crosswalks.iter().take(i as usize)
                    .all(|p| (*p as i32 - perim_loc as i32).abs() > CROSSWALK_SPACING as i32) {
                break;
            }
            attempts_i += 1;
            if attempts_i > ATTEMPTS {
                return; // probably constrained to not get all of them
            }
        }
        chosen_crosswalks[i as usize] = perim_loc;

        // go through sections, top, right, bottom, left...
        let left1 = BUILDING_WIDTH + m.vert_sidewalk_width;
        let left2 = BUILDING_WIDTH + m.vert_sidewalk_width + m.vert_road_width;
        let top1 = BUILDING_WIDTH + m.horiz_sidewalk_width;
        let top2 = BUILDING_WIDTH + m.horiz_sidewalk_width + m.horiz_road_width;
        let right = left1 + m.horiz_road_length - m.vert_road_width;
        let bottom = top1 + m.vert_road_length - m.horiz_road_width;
        if perim_loc < horiz_crosswalk_perim {
            fill_map_rect(&mut m.grid[..], left2 + perim_loc, top1,
                                           CROSSWALK_WIDTH, m.horiz_road_width, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - horiz_crosswalk_perim;
        if perim_loc < vert_crosswalk_perim {
            fill_map_rect(&mut m.grid[..], right, top2 + perim_loc,
                                           m.vert_road_width, CROSSWALK_WIDTH, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - vert_crosswalk_perim;
        if perim_loc < horiz_crosswalk_perim {
            fill_map_rect(&mut m.grid[..], left2 + perim_loc, bottom,
                                           CROSSWALK_WIDTH, m.horiz_road_width, Cell::CROSSWALK);
            continue;
        }
        let perim_loc = perim_loc - horiz_crosswalk_perim;
        fill_map_rect(&mut m.grid[..], left1, top2 + perim_loc,
                                       m.vert_road_width, CROSSWALK_WIDTH, Cell::CROSSWALK);
    }
}

fn create_map() -> WorldMap {
    // The world will have a single block of road, making a rectangle.
    let grid = [Cell{bits: 0u8}; MAP_SIZE as usize];
    let buildings = Vec::new();

    let mut rng = repeatable_rand(); //rand::thread_rng();

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
    println!("horiz sidewalk width: {} vert sidewalk width: {}",
              horiz_sidewalk_width, vert_sidewalk_width);

    let mut map = WorldMap{grid: Box::new(grid), buildings, grid_width: MAP_WIDTH,
                             horiz_road_width, vert_road_width,
                             horiz_sidewalk_width, vert_sidewalk_width,
                             horiz_road_length, vert_road_length,
                             pedestrian_inner_pts: [(0, 0); 4],
                             pedestrian_outer_pts: [(0, 0); 4],
                             vehicle_inner_pts: [(0, 0); 4],
                             vehicle_outer_pts: [(0, 0); 4],
                             vehicle_inner_rects: [[(0.0, 0.0); 4]; 4],
                             vehicle_outer_rects: [[(0.0, 0.0); 4]; 4],
                             corners: [((0, 0), (0, 0)); 4]};

    // mark the grid accordingly
    // horizontal roads, both top and bottom
    {
        let left = BUILDING_WIDTH + vert_sidewalk_width;
        let top_1 = BUILDING_WIDTH + horiz_sidewalk_width;
        let top_2 = top_1 + vert_road_length - horiz_road_width;
        fill_map_rect(&mut map.grid[..], left, top_1,
                                     horiz_road_length, horiz_road_width/2,
                                     Cell::OUTER_LANE);
        map.vehicle_outer_rects[0] = axis_rect_to_rect_f32(
                                        left, top_1, horiz_road_length, horiz_road_width/2);

        fill_map_rect(&mut map.grid[..], left + vert_road_width/2, top_1 + horiz_road_width/2,
                                     horiz_road_length - vert_road_width, horiz_road_width/2,
                                     Cell::INNER_LANE);
        map.vehicle_inner_rects[0] = axis_rect_to_rect_f32(
                                        left + vert_road_width/2, top_1 + horiz_road_width/2,
                                        horiz_road_length - vert_road_width, horiz_road_width/2);

        fill_map_rect(&mut map.grid[..], left + vert_road_width/2, top_2,
                                     horiz_road_length - vert_road_width, horiz_road_width/2,
                                     Cell::INNER_LANE);
        map.vehicle_inner_rects[2] = axis_rect_to_rect_f32(
                                        left + vert_road_width/2, top_2,
                                        horiz_road_length - vert_road_width, horiz_road_width/2);

        fill_map_rect(&mut map.grid[..], left, top_2 + horiz_road_width/2,
                                     horiz_road_length, horiz_road_width/2,
                                     Cell::OUTER_LANE);
        map.vehicle_outer_rects[2] = axis_rect_to_rect_f32(
                                        left, top_2 + horiz_road_width/2,
                                        horiz_road_length, horiz_road_width/2);

        let left_2 = left + horiz_road_length - vert_road_width;
        map.corners[0] = bound_rect_u32_to_i32(((left, top_1), (left + vert_road_width, top_1 + horiz_road_width)));
        map.corners[1] = bound_rect_u32_to_i32(((left_2, top_1), (left_2 + vert_road_width, top_1 + horiz_road_width)));
        map.corners[2] = bound_rect_u32_to_i32(((left_2, top_2), (left_2 + vert_road_width, top_2 + horiz_road_width)));
        map.corners[3] = bound_rect_u32_to_i32(((left, top_1), (left + vert_road_width, top_1 + horiz_road_width)));
    }

    // vertical roads
    {
        let top = BUILDING_WIDTH + horiz_sidewalk_width;
        let left_1 = BUILDING_WIDTH + vert_sidewalk_width;
        let left_2 = left_1 + horiz_road_length - vert_road_width;
        fill_map_rect(&mut map.grid[..], left_1, top,
                                     vert_road_width/2, vert_road_length,
                                     Cell::OUTER_LANE);
        map.vehicle_outer_rects[3] = axis_rect_to_rect_f32(
                                        left_1, top, vert_road_width/2, vert_road_length);

        fill_map_rect(&mut map.grid[..], left_1 + vert_road_width/2, top + horiz_road_width/2,
                                    vert_road_width/2, vert_road_length - horiz_road_width,
                                    Cell::INNER_LANE);
        map.vehicle_inner_rects[3] = axis_rect_to_rect_f32(
                                        left_1 + vert_road_width/2, top + horiz_road_width/2,
                                        vert_road_width/2, vert_road_length - horiz_road_width);

        fill_map_rect(&mut map.grid[..], left_2, top + horiz_road_width/2,
                                     vert_road_width/2, vert_road_length - horiz_road_width,
                                     Cell::INNER_LANE);
        map.vehicle_inner_rects[1] = axis_rect_to_rect_f32(
                                        left_2, top + horiz_road_width/2,
                                        vert_road_width/2, vert_road_length - horiz_road_width);

        fill_map_rect(&mut map.grid[..], left_2 + vert_road_width/2, top,
                                     vert_road_width/2, vert_road_length,
                                     Cell::OUTER_LANE);
        map.vehicle_outer_rects[1] = axis_rect_to_rect_f32(
                                        left_2 + vert_road_width/2, top,
                                        vert_road_width/2, vert_road_length,);
    }

    create_buildings(&mut map);
    create_crosswalks(&mut map);

    // walls on the boundary
    fill_map_rect(&mut map.grid[..], 0, 0, MAP_WIDTH, 1, Cell::OBSTACLE);
    fill_map_rect(&mut map.grid[..], MAP_WIDTH - 1, 0, 1, MAP_HEIGHT, Cell::OBSTACLE);
    fill_map_rect(&mut map.grid[..], 0, MAP_HEIGHT - 1, MAP_WIDTH, 1, Cell::OBSTACLE);
    fill_map_rect(&mut map.grid[..], 0, 0, 1, MAP_HEIGHT, Cell::OBSTACLE);

    map
}

fn axis_rect_to_rect_f32(left: u32, top: u32, width: u32, height: u32) -> RectF32 {
    let (left, top) = (left as f32, top as f32);
    let (width, height) = (width as f32, height as f32);

    [(left, top), (left + width, top),
     (left + width, top + height), (left, top + height)]
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Hash)]
enum AgentKind {
    Obstacle,
    Pedestrian,
    Vehicle,
}

impl Default for AgentKind {
    fn default() -> AgentKind { AgentKind::Obstacle }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Agent {
    kind: AgentKind,
    on_map: bool,
    frozen_steps: u32, // does not get an action for x steps... like after a collision
    // number of consecutive explore actions taken (after getting to only explore_one_in_max)
    explore_actions: u32,

    pose: (u32, u32, u32), // x, y, theta (increments of THETA_PER_INC)
    width: u32, // perpendicular to theta
    length: u32, // in direction of theta
    velocity: i32,
    health: i32,

    source_building_i: u32,
    destination_building_i: u32,

    has_deadline: bool,
    deadline_reward: f32,
    timestep_cost: f32,
    risk_cost: f32,
    folk_theory: f32,
    habit_theory: f32,
    internalization_theory: f32,
    choice_restriction_theory: bool,
}

impl Eq for Agent {}

// we only serialize and send the pose (and whether agent is on map)
impl serde::Serialize for Agent {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where S: serde::Serializer
    {
        let mut state = serializer.serialize_struct("Agent", 9)?;
        state.serialize_field("kind", &self.kind)?;
        state.serialize_field("on_map", &self.on_map)?;
        state.serialize_field("x", &self.pose.0)?;
        state.serialize_field("y", &self.pose.1)?;
        state.serialize_field("theta", &(self.pose.2 as f32 * THETA_PER_INC))?;
        state.serialize_field("width", &self.width)?;
        state.serialize_field("length", &self.length)?;
        // feed the gui a value 0 to 99
        let health = self.health * 99 / C.total_health as i32;
        state.serialize_field("health", &health)?;
        state.serialize_field("destination_building_i", &self.destination_building_i)?;
        state.end()
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
struct WorldStats {
    collisions: u32,
    deaths: u32,
    trips_completed: u32,
    timesteps: u32,
}

#[derive(Clone, Serialize)]
struct WorldStateInner {
    map: WorldMap,
    agents: Vec<Agent>,
    stats: WorldStats,
}

type WorldState = Arc<Mutex<WorldStateInner>>;

type LineU32 = ((u32, u32), (u32, u32));
type LineF32 = ((f32, f32), (f32, f32));
type RectU32 = [(u32, u32); 4];
type RectF32 = [(f32, f32); 4];

fn points_to_lines_f32(pts: &RectF32) -> [LineF32; 4] {
    [(pts[0], pts[1]), (pts[1], pts[2]), (pts[2], pts[3]), (pts[3], pts[0])]
}

fn points_to_lines_u32(pts: &RectU32) -> [LineU32; 4] {
    [(pts[0], pts[1]), (pts[1], pts[2]), (pts[2], pts[3]), (pts[3], pts[0])]
}

fn point_u32_to_f32(pt: (u32, u32)) -> (f32, f32) {
    (pt.0 as f32, pt.1 as f32)
}

fn points_u32_to_lines_f32(pts: &RectU32) -> [LineF32; 4] {
    let pts = [point_u32_to_f32(pts[0]), point_u32_to_f32(pts[1]),
               point_u32_to_f32(pts[2]), point_u32_to_f32(pts[3])];
    [(pts[0], pts[1]), (pts[1], pts[2]), (pts[2], pts[3]), (pts[3], pts[0])]
}

// manhattan distance, the 1 norm
// fn dist_1(pts: ((u32, u32), (u32, u32))) -> u32 {
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

type BoundingRectF32 = ((f32, f32), (f32, f32));
type BoundingRectU32 = ((u32, u32), (u32, u32));
type BoundingRectI32 = ((i32, i32), (i32, i32));

fn pose_size_to_bounding_rect_f32(p: (i32, i32, i32), size: (f32, f32))
                              -> BoundingRectF32 {
    let (x, y, t) = p;
    let t = t as i32;
    let (sx, sy) = size;
    let (x, y) = (x as f32, y as f32);

    let rw = rotate_pt((sx, 0.0), -t);
    let rl = rotate_pt((0.0, sy), -t);

    let min_x = x.min(x + rw.0).min(x + rl.0).min(x + rw.0 + rl.0);
    let min_y = y.min(y + rw.1).min(y + rl.1).min(y + rw.1 + rl.1);
    let max_x = x.max(x + rw.0).max(x + rl.0).max(x + rw.0 + rl.0);
    let max_y = y.max(y + rw.1).max(y + rl.1).max(y + rw.1 + rl.1);

    ((min_x, min_y), (max_x, max_y))
}

fn bound_rect_u32_to_i32(bounds: BoundingRectU32) -> BoundingRectI32 {
    let (mins, maxs) = bounds;
    ((mins.0 as i32, mins.1 as i32),
     (maxs.0 as i32, maxs.1 as i32))
}

fn bound_rect_f32_to_i32(bounds: BoundingRectF32) -> BoundingRectI32 {
    let (mins, maxs) = bounds;
    ((mins.0.floor() as i32, mins.1.floor() as i32),
     (maxs.0.ceil() as i32, maxs.1.ceil() as i32))
}

fn pose_size_to_bounding_rect(p: (u32, u32, u32), size: (u32, u32))
                              -> BoundingRectI32 {
    let bounds = pose_size_to_bounding_rect_f32((p.0 as i32, p.1 as i32, p.2 as i32),
                                                (size.0 as f32, size.1 as f32));
    bound_rect_f32_to_i32(bounds)
}

fn create_bounding_rect_table_for_size(size: (u32, u32)) -> Vec<BoundingRectI32> {
    let mut v = Vec::with_capacity((MAP_WIDTH * MAP_HEIGHT * TWO_PI_INCS) as usize);
    for x in 0..(MAP_WIDTH + 1) {
        for y in 0..(MAP_HEIGHT + 1) {
            for t in 0..TWO_PI_INCS {
                let rect = pose_size_to_bounding_rect((x, y, t), size);
                v.push(rect);
            }
        }
    }
    v
}

lazy_static! {
    static ref PEDESTRIAN_BOUNDING_RECT_TABLE: Vec<BoundingRectI32> = {
        create_bounding_rect_table_for_size((PEDESTRIAN_SIZE, PEDESTRIAN_SIZE))
    };
    static ref VEHICLE_BOUNDING_RECT_TABLE: Vec<BoundingRectI32> = {
        create_bounding_rect_table_for_size((VEHICLE_WIDTH, VEHICLE_LENGTH))
    };
    static ref BUILDING_BOUNDING_RECT_TABLE: Vec<BoundingRectI32> = {
        let mut v = Vec::with_capacity((MAP_WIDTH * MAP_HEIGHT) as usize);
        for x in 0..(MAP_WIDTH + 1) {
            for y in 0..(MAP_HEIGHT + 1) {
                let rect = pose_size_to_bounding_rect((x, y, 0),
                                                      (BUILDING_WIDTH, BUILDING_WIDTH));
                v.push(rect);
            }
        }
        v
    };
    static ref SIN_COS_TABLE: [(f32, f32); TWO_PI_INCS as usize] = {
        let mut table = [(0.0, 0.0); TWO_PI_INCS as usize];
        for theta in 0..TWO_PI_INCS {
            table[theta as usize] = (theta as f32 * THETA_PER_INC).sin_cos();
        }
        table
    };
}

fn fast_sin_cos(theta_incs: i32) -> (f32, f32) {
    if theta_incs < 0 {
        SIN_COS_TABLE[(TWO_PI_INCS as i32 + theta_incs) as usize]
    } else {
        SIN_COS_TABLE[theta_incs as usize]
    }
    // (theta_incs as f32 * THETA_PER_INC).sin_cos()
}

#[inline(always)]
fn lookup_bounding_rect(p: (u32, u32, u32), size: (u32, u32))
                              -> BoundingRectI32 {
    if size.0 == PEDESTRIAN_SIZE && size.1 == PEDESTRIAN_SIZE {
        let i = p.2 + p.1 * TWO_PI_INCS + p.0 * (TWO_PI_INCS * (MAP_HEIGHT + 1));
        return PEDESTRIAN_BOUNDING_RECT_TABLE[i as usize];
    }
    if size.0 == VEHICLE_WIDTH && size.1 == VEHICLE_LENGTH {
        let i = p.2 + p.1 * TWO_PI_INCS + p.0 * (TWO_PI_INCS * (MAP_HEIGHT + 1));
        return VEHICLE_BOUNDING_RECT_TABLE[i as usize];
    }
    if size.0 == BUILDING_WIDTH && size.1 == BUILDING_WIDTH {
        let i = p.1 + p.0 * (MAP_HEIGHT + 1);
        return BUILDING_BOUNDING_RECT_TABLE[i as usize];
    }
    // should be just walls left
    if size.1 == MAP_WIDTH && p.1 == 1 {
        return ((0, 0), (MAP_WIDTH as i32, 1)); // top
    } else if size.1 == MAP_HEIGHT && p.0 == MAP_WIDTH - 1 {
        return ((MAP_WIDTH as i32 - 1, 0), (MAP_WIDTH as i32, MAP_HEIGHT as i32)); // right
    } else if size.1 == MAP_WIDTH && p.1 == MAP_HEIGHT {
        return ((0, MAP_HEIGHT as i32 - 1), (MAP_WIDTH as i32, MAP_HEIGHT as i32)); // bottom
    } else if size.1 == MAP_HEIGHT {
        return ((0, 0), (1, MAP_HEIGHT as i32)); // left
    }
    pose_size_to_bounding_rect(p, size)
}

// manhattan distance/1 norm, considers that the positions are upper-left corners
// and that the objects have their sizes expanding to the right and down.
// uses the bounding box of the rotated objects since this is mainly intended for
// dealing only with rotations that are multiples of 90 degrees.
// xxx yyy = 1
// xxx
//   yyy = 0 (-1)
#[inline(always)]
fn obj_dist_1(a: (u32, u32, u32), size_a: (u32, u32),
              b: (u32, u32, u32), size_b: (u32, u32)) -> u32
{
    // l for low (min) and h for high (max)
    let ((lx1, ly1), (hx1, hy1)) = lookup_bounding_rect(a, size_a);
    let ((lx2, ly2), (hx2, hy2)) = lookup_bounding_rect(b, size_b);
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
    (x_diff + y_diff) as u32
}

fn calc_total_perim(pts: &RectU32) -> u32 {
    let lines = points_to_lines_u32(pts);
    lines.iter().map(|&a| dist_1(a)).sum()
}

fn perim_i_to_pose(m: &WorldMap, perim_i: u32, perims: &Vec<u32>, lines: &[LineU32; 4],
                   size: (u32, u32), is_inner: bool) -> Option<(u32, u32, u32)> {
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
    let pt = ((x1 as i32 + sx * perim_val as i32) as u32,
              (y1 as i32 + sy * perim_val as i32) as u32);
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
            return Some((pt.0 + 1, pt.1 - size.0, TWO_PI_INCS - PI_OVER_TWO_INCS));
        } else if line_i == 1 {
            if pt.1 < y1 + size.1 - m.horiz_road_width / 2 {
                return None;
            }
            return Some((pt.0 + size.0, pt.1 + 1, PI_INCS));
        } else if line_i == 2 {
            if pt.0 > x1 - size.1 + m.vert_road_width / 2 {
                return None;
            }
            return Some((pt.0, pt.1 + size.0, PI_OVER_TWO_INCS));
        } else if line_i == 3 {
            if pt.1 > y1 - size.1 + m.horiz_road_width / 2 {
                return None;
            }
            return Some((pt.0 - size.0, pt.1, 0));
        } else {
            panic!("Should not be possible");
        }
    } else {
        if line_i == 0 {
            if pt.0 > x2 - size.1 {
                return None;
            }
            return Some((pt.0, pt.1 + size.0, PI_OVER_TWO_INCS));
        } else if line_i == 1 {
            if pt.1 > y2 - size.1 {
                return None;
            }
            return Some((pt.0 - size.0, pt.1, 0));
        } else if line_i == 2 {
            if pt.0 < x2 + size.1 {
                return None;
            }
            return Some((pt.0, pt.1 - size.0, TWO_PI_INCS - PI_OVER_TWO_INCS));
        } else if line_i == 3 {
            if pt.1 < y2 + size.1 {
                return None;
            }
            return Some((pt.0 + size.0, pt.1, PI_INCS));
        } else {
            panic!("Should not be possible");
        }
    }
}

// tries to find a "spawn" pose (x, y, theta) for agent with size (width, length)
// within building_margin of a building along one of the two perimeters,
// margin away from all existing agents
// makes the assumption that points form a strict rectangle (only y or x change at a time)
fn spawn_from_perim(m: &WorldMap, building: Building, building_margin: u32,
                    size: (u32, u32), pts: &RectU32, occupied: &Vec<bool>,
                    margin: u32, is_inner: bool)
                    -> Option<(u32, u32, u32)> {
    let mut rng = repeatable_rand(); //rand::thread_rng();

    let lines = points_to_lines_u32(pts);

    let perims = lines.iter().map(|&a| dist_1(a));
    let perims = perims.collect::<Vec<_>>();
    let perim_total = perims.iter().sum();

    let perim_loc;
    {
        let perim_choices = 0..perim_total;
        let margin = margin + size.1; // include agent length in margin
        let perim_choices = perim_choices.filter(|&i|
                                a_to_b_mod(i + perim_total - margin, i + margin + 1, perim_total)
                                .iter().all(|&j| !occupied[j as usize]));
        let perim_choices = perim_choices.filter(|&i| {
            let pose = perim_i_to_pose(m, i, &perims, &lines, size, is_inner);
            if pose.is_none() {
                return false;
            }
            let pose = pose.unwrap();
            obj_dist_1((building.x, building.y, 0),
                       (BUILDING_WIDTH, BUILDING_WIDTH), pose, size) <= building_margin
        });
        let perim_choices = perim_choices.collect::<Vec<_>>();

        // let mut min_d = u32::max_value();
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
    // let dist = obj_dist_1((building.x, building.y, 0.0),
    //                       (BUILDING_WIDTH, BUILDING_WIDTH), pose, size);
    // println!("Chose perim_loc: {} at dist {} from building {:?}, with pose {:?}",
    //          perim_loc, dist, building, pose);

    Some(pose)
}

// tries to find a "spawn" pose (x, y, theta) for agent with size (width, length)
// within building_margin of a building along one of the two perimeters,
// margin away from all existing agents
// returns (pose, building_i)
fn spawn_on_two_perims(m: &WorldMap, buildings: &[Building], building_margin: u32,
                       size: (u32, u32),
                       pts_a: &RectU32, pts_b: &RectU32,
                       occupied_a: &Vec<bool>, occupied_b: &Vec<bool>,
                       margin: u32) -> Option<((u32, u32, u32), u32)> {
    let mut rng = repeatable_rand(); //rand::thread_rng();

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
            return Some((res.unwrap(), i as u32));
        }
    }
    None
}

fn line_to_normal(line: &LineF32) -> f32 {
    ((line.1).1 - (line.0).1).atan2((line.1).0 - (line.0).0)
}

fn lines_to_normals(lines: &[LineF32; 4]) -> [f32; 4] {
    [line_to_normal(&lines[0]), line_to_normal(&lines[1]),
     line_to_normal(&lines[2]), line_to_normal(&lines[3])]
}

fn rotate_pt(pt: (f32, f32), angle: i32) -> (f32, f32) {
    let (x, y) = (pt.0, pt.1);
    let (sin_a, cos_a) = fast_sin_cos(angle);
    (cos_a * x - sin_a * y, sin_a * x + cos_a * y)
}

// project rectangle onto axis defined by angle and then return (min, max)
fn project_rect(rect: &RectF32, angle: i32) -> (f32, f32) {
    let mut min_val = rotate_pt(rect[0], angle).0;
    let mut max_val = min_val;
    for i in 1..4 {
        let new_val = rotate_pt(rect[i], angle).0;
        if new_val < min_val {
            min_val = new_val;
        } else if new_val > max_val {
            max_val = new_val;
        }
    }
    (min_val, max_val)
}

fn overlaps_rect_check_normals(q1: &RectF32, q2: &RectF32, normals: &[f32; 4]) -> bool {
    for normal in normals {
        let normal = ((normal / THETA_PER_INC + TWO_PI_INCS as f32 + 0.5) as u32) % TWO_PI_INCS;
        let normal = normal as i32;

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

fn overlaps_rect(q1: &RectF32, q2: &RectF32) -> bool {
    // apply the separating axis theorem
    // we loops through the 2 axes of q1, then those of q2
    let q1_lines = points_to_lines_f32(q1);
    let normals1 = lines_to_normals(&q1_lines);
    let q2_lines = points_to_lines_f32(q2);
    let normals2 = lines_to_normals(&q2_lines);
    if !overlaps_rect_check_normals(q1, q2, &normals1) {
        return false;
    }
    overlaps_rect_check_normals(q1, q2, &normals2)
}

fn agent_to_rect32(agent: &Agent) -> RectF32 {
    let (width, length) = (agent.width as f32, agent.length as f32);
    let (x, y, theta) = (agent.pose.0 as f32, agent.pose.1 as f32,
                         agent.pose.2 as i32);
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
fn overlaps_path(agent: &Agent, path: &RectU32,
                 path_width: u32, is_inner: bool) -> Vec<u32> {
    let path_width = path_width as f32;
    let agent_rect = agent_to_rect32(agent);

    let mut overlap_locs = Vec::new();
    let path_lines = points_u32_to_lines_f32(path);
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
            overlap_locs.push(perim_loc + perim_pos as u32);
        }
        perim_loc += dist_1(line) as u32;
    }

    overlap_locs
}

// range like [a, b), but can go by +1 or -1
fn a_to_b(a: u32, b: u32) -> Vec<u32> {
    if a < b {
        return (a..b).collect();
    } else {
        return ((b+1)..(a+1)).rev().collect();
    }
}

// range like [a, b), but can wraps around modular boundary if a > b
fn a_to_b_mod(a: u32, b: u32, n: u32) -> Vec<u32> {
    let a = a % n;
    let b = b % n;
    if a < b {
        return (a..b).collect();
    } else {
        return (a..n).chain(0..b).collect();
    }
}

fn mod_dist(a: u32, b: u32, n: u32) -> u32 {
    // let a = a % n;
    // let b = b % n;
    difference((a + n * 3 / 2 - b) % n, n / 2)
}

// assumes path is in order of top, right, bottom, left.
fn calc_occupied_perim_map(agents: &[Agent], path: &RectU32,
                           horiz_width: u32, vert_width: u32, is_inner: bool) -> Vec<bool>
{
    let path_lines = points_to_lines_u32(path);
    let perim_total = calc_total_perim(path);
    let mut occupied_perims = vec![false; perim_total as usize];

    let (horiz_width, vert_width) = (horiz_width as f32, vert_width as f32);

    let mut path_cell_rects = Vec::new();
    for (i, &line) in path_lines.iter().enumerate() {
        let ((x1, y1), (x2, y2)) = line;
        if x1 == x2 {
            let path_width =
                if (i == 1 && !is_inner) ||
                   (i == 3 && is_inner) {
                    -vert_width
                } else {
                    vert_width
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
                    -horiz_width
                } else {
                    horiz_width
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

        let path_width = horiz_width.max(vert_width) as u32;
        let overlap_locs = overlaps_path(agent, path, path_width, is_inner);
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
                let overlap_loc_dist_sq = mod_dist(perim_loc as u32, overlap_loc, perim_total).pow(2);
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
    let mut rng = repeatable_rand(); //rand::thread_rng();
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
    let mut rng = repeatable_rand(); //rand::thread_rng();

    let mut p_count = agents.iter().filter(|p|
                                            p.kind == AgentKind::Pedestrian && p.on_map).count();
    while p_count < C.pedestrian_n {
        let agent_i = draw_off_map_agent_i(&agents, AgentKind::Pedestrian);
        if agent_i.is_none() {
            break; // no agents left to draw
        }
        let agent_i = agent_i.unwrap();

        let on_outer = calc_occupied_perim_map(agents, &m.pedestrian_outer_pts,
                                            m.horiz_sidewalk_width, m.vert_sidewalk_width, false);
        let on_inner = calc_occupied_perim_map(agents, &m.pedestrian_inner_pts,
                                            m.horiz_sidewalk_width, m.vert_sidewalk_width, true);

        // println!("On outer open: {:?}", on_outer.iter().filter(|&b| !b).count());
        // println!("On inner open: {:?}", on_inner.iter().filter(|&b| !b).count());

        let spawn_result = spawn_on_two_perims(m, &m.buildings, BUILDING_PEDESTRIAN_MARGIN,
                                               (PEDESTRIAN_SIZE, PEDESTRIAN_SIZE),
                                               &m.pedestrian_outer_pts, &m.pedestrian_inner_pts,
                                               &on_outer, &on_inner, SPAWN_MARGIN);

        if let Some((pose, building_i)) = spawn_result {
            let mut agent = &mut agents[agent_i as usize];
            // println!("Drew pose: {:?} for pedestrian\n", pose);
            agent.on_map = true;
            agent.pose = pose;
            agent.source_building_i = building_i;
            agent.destination_building_i = building_i;
            while agent.destination_building_i == building_i {
                agent.destination_building_i = rng.gen_range(0, m.buildings.len() as u32);
            }
            p_count += 1;
        } else {
            // no locations are open even though we want to spawn another agent.
            // Just try again later.
            break;
        }
    }
}

fn replenish_vehicles(m: &WorldMap, agents: &mut [Agent]) {
    let mut rng = repeatable_rand(); //rand::thread_rng();

    let mut v_count = agents.iter().filter(|p| p.kind == AgentKind::Vehicle && p.on_map).count();

    while v_count < C.vehicle_n {
        let agent_i = draw_off_map_agent_i(&agents, AgentKind::Vehicle);
        if agent_i.is_none() {
            break; // no agents left to draw
        }
        let agent_i = agent_i.unwrap();

        let on_outer = calc_occupied_perim_map(agents, &m.vehicle_outer_pts,
                                                    m.horiz_road_width, m.vert_road_width, false);
        let on_inner = calc_occupied_perim_map(agents, &m.vehicle_inner_pts,
                                                    m.horiz_road_width, m.vert_road_width, true);

        // println!("On outer: {:?}", on_outer.iter().filter(|&b| !b).count());
        // println!("On inner: {:?}", on_inner.iter().filter(|&b| !b).count());

        let spawn_result = spawn_on_two_perims(m, &m.buildings, BUILDING_VEHICLE_MARGIN,
                                               (VEHICLE_WIDTH, VEHICLE_LENGTH),
                                               &m.vehicle_outer_pts, &m.vehicle_inner_pts,
                                               &on_outer, &on_inner, SPAWN_MARGIN);

        if let Some((pose, building_i)) = spawn_result {
            let mut agent = &mut agents[agent_i];
            // println!("Drew pose: {:?} for vehicle\n", pose);
            agent.on_map = true;
            agent.pose = pose;
            agent.source_building_i = building_i;
            agent.destination_building_i = building_i;
            while agent.destination_building_i == building_i {
                agent.destination_building_i = rng.gen_range(0, m.buildings.len() as u32);
            }
            v_count += 1;
        } else {
            // no locations are open even though we want to spawn another agent.
            // Just try again later.
            break;
        }
    }
}

fn create_edge_walls() -> Vec<Agent> {
    let mut agents = Vec::new();

    // edges of the map; top, right, bottom, left
    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: 1, length: MAP_WIDTH,
                     pose: (0, 1, PI_OVER_TWO_INCS),
                     ..Agent::default()};
    agents.push(edge);

    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: 1, length: MAP_HEIGHT,
                     pose: (MAP_WIDTH - 1, 0, 0),
                     ..Agent::default()};
    agents.push(edge);

    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: 1, length: MAP_WIDTH,
                     pose: (0, MAP_HEIGHT, PI_OVER_TWO_INCS),
                     ..Agent::default()};
    agents.push(edge);

    let edge = Agent{kind: AgentKind::Obstacle, on_map: true,
                     width: 1, length: MAP_HEIGHT,
                     pose: (0, 0, 0),
                     ..Agent::default()};
    agents.push(edge);
    agents
}

fn building_i_to_agent_i(building_i: u32) -> usize {
    building_i as usize + C.population_n
}

fn create_agents(map: &mut WorldMap) -> Vec<Agent> {
    debug_assert!((C.population_n % 2) == 0);

    let mut agents = Vec::new();
    if C.use_single_agent_type {
        let p = Agent{kind: AgentKind::Pedestrian,
                      width: PEDESTRIAN_SIZE, length: PEDESTRIAN_SIZE,
                      health: C.total_health as i32,
                      habit_theory: C.habit_theory,
                      folk_theory: C.folk_theory,
                      internalization_theory: C.internalization_theory,
                      choice_restriction_theory: C.choice_restriction_theory,
                      ..Agent::default()};
        for _ in 0..(C.population_n/2) {
            agents.push(p.clone());
        }

        let v = Agent{kind: AgentKind::Vehicle,
                      width: VEHICLE_WIDTH, length: VEHICLE_LENGTH,
                      health: C.total_health as i32,
                      habit_theory: C.habit_theory,
                      folk_theory: C.folk_theory,
                      internalization_theory: C.internalization_theory,
                      choice_restriction_theory: C.choice_restriction_theory,
                      ..Agent::default()};
        for _ in 0..(C.population_n/2) {
            agents.push(v.clone());
        }
    }

    // add buildings
    let b = Agent{kind: AgentKind::Obstacle, on_map: true,
                  width: BUILDING_WIDTH, length: BUILDING_WIDTH,
                  ..Agent::default()};
    for building in &mut map.buildings {
        let mut b_agent = b.clone();
        b_agent.pose = (building.x, building.y, 0);

        building.agent_i = agents.len();
        agents.push(b_agent);
    }

    let mut walls = create_edge_walls();
    agents.extend(walls.drain(..));

    agents
}

fn setup_agents(map: &WorldMap, agents: &mut [Agent]) {
    replenish_pedestrians(map, agents);
    replenish_vehicles(map, agents);
}

#[derive(Debug, Clone, Copy, Rand, PartialEq, Eq, Hash)]
enum Action {
    TurnLeft,
    TurnRight,
    AccelerateForward,
    AccelerateBackward,
    Brake,
    Continue,
    Frozen,
    Nothing,
}

impl Action {
    pub fn normal_actions() -> &'static [Action] {
        static ACTIONS: [Action; 6] = [Action::TurnLeft, Action::TurnRight,
                                       Action::AccelerateForward, Action::AccelerateBackward,
                                       Action::Brake, Action::Continue];
        &ACTIONS
    }
}

#[derive(Debug, Eq, PartialEq, Hash, Clone, Copy)]
struct QStateAvoid {
    vel: i32,
    is_pedestrian: bool,
    // other_pedestrian: bool,
    other_x: i32,
    other_y: i32,
    // other_theta: u32,
    // other_vel: i32,
}

#[derive(Debug, Eq, PartialEq, Hash, Clone, Copy)]
struct QStateTarget {
    is_pedestrian: bool,
    x: u32,
    y: u32,
    theta: u32,
    vel: i32,
    destination_building_i: u32,
}

fn qstate_avoid_for_agent(agent: &Agent, other: &Agent) -> QStateAvoid {
    let (sin_a, cos_a) = fast_sin_cos(-(agent.pose.2 as i32));
    let dx = (other.pose.0 as f32) - (agent.pose.0 as f32);
    let dy = (other.pose.1 as f32) - (agent.pose.1 as f32);

    let (a_width, a_length) = (agent.width as f32, agent.length as f32);

    // create bounding rect for the other based on the relative angles
    // we also separately need to rotate the vector (dx, dy) by the agent's theta
    // this way we can construct the other's full pose in the agent's body frame

    let rel_x = cos_a * dx + sin_a * dy;
    let rel_y = -sin_a * dx + cos_a * dy;

    // account for the size of the (main) agent, which is assumed to be as if at theta=0
    let rel_x = if rel_x > 0.0 {
                    if a_width >= rel_x { 0.0 } else { rel_x - a_width} } else { rel_x };
    let rel_y = if rel_y > 0.0 {
                    if a_length >= rel_y { 0.0 } else { rel_y - a_length} } else { rel_y };

    let rel_theta = (other.pose.2 + TWO_PI_INCS - agent.pose.2) % TWO_PI_INCS;
    let (o_width, o_length) = (other.width as f32, other.length as f32);
    let ((lx, ly), (hx, hy)) = pose_size_to_bounding_rect_f32(
                                            (0, 0, rel_theta as i32),
                                            (o_width, o_length));
    let ((lx, ly), (hx, hy)) = ((lx + rel_x,
                                ly + rel_y),
                                (hx + rel_x,
                                hy + rel_y));

    let rel_x = if lx * hx <= 0.0 { 0.0 } else {
        if lx.abs() < hx.abs() { lx } else { hx }
    };
    let rel_y = if ly * hy <= 0.0 { 0.0 } else {
        if ly.abs() < hy.abs() { ly } else { hy }
    };
    let (rel_x, rel_y) = (rel_x.round() as i32, rel_y.round() as i32);

    QStateAvoid {
                  is_pedestrian: agent.kind == AgentKind::Pedestrian,
                  // other_pedestrian: other.kind == AgentKind::Pedestrian,
                  vel: agent.velocity.signum(),
                  other_x: rel_x,
                  other_y: rel_y,
                  // other_theta: rel_theta,
                  // other_vel: other.velocity,
                }
}

fn qstate_target_for_agent(agent: &Agent) -> QStateTarget {
    QStateTarget {
                   is_pedestrian: agent.kind == AgentKind::Pedestrian,
                   x: agent.pose.0, y: agent.pose.1, theta: agent.pose.2, vel: agent.velocity,
                   destination_building_i: agent.destination_building_i,
                 }
}

struct QLearning {
    avoid_values: FnvHashMap<QStateAvoid, Vec<f32>>,
    target_values: FnvHashMap<QStateTarget, Vec<f32>>,
    avoid_empties: u32,
}

impl Default for QLearning {
    fn default() -> QLearning {
        QLearning {
            avoid_values: FnvHashMap::default(),
            target_values: FnvHashMap::default(),
            avoid_empties: 0,
        }
    }
}

fn q_target_weights(q: &mut QLearning, select_actions: &[Action], agent: &Agent)
                    -> Vec<f32> {
    let mut weights = vec![0.0; select_actions.len()];
    let qstate_target = qstate_target_for_agent(agent);
    let q_target_vals = q.target_values.get(&qstate_target);
    if let Some(q_target_vals) = q_target_vals {
        let all_actions = Action::normal_actions();

        let mut weight_i = 0;
        for (select_i, &a) in select_actions.iter().enumerate() {
            while all_actions[weight_i] != a {
                weight_i += 1;
            }
            weights[select_i] += q_target_vals[weight_i];
            weight_i += 1;
        }
    }
    weights
}

fn q_avoid_weights(q: &mut QLearning, agents: &[Agent],
                   agent_dists: &AgentDistanceTable,
                   select_actions: &[Action], agent_i: usize) -> Vec<f32> {
    let agent = &agents[agent_i];
    let all_actions = Action::normal_actions();
    let mut weights = vec![0.0; select_actions.len()];

    let mut num_summed = 0;
    for other_i in 0..agents.len() {
        let dist = get_agent_1_dist(&agent_dists, agent_i, other_i);
        if dist <= C.state_avoid_dist {
            num_summed += 1;
            let qstate_avoid = qstate_avoid_for_agent(agent, &agents[other_i]);
            let more_weights = q.avoid_values.get(&qstate_avoid);
            if let Some(more_weights) = more_weights {
                // put the weights into the correct place for the selected actions
                let mut weight_i = 0;
                for (select_i, &a) in select_actions.iter().enumerate() {
                    while all_actions[weight_i] != a {
                        weight_i += 1;
                    }
                    weights[select_i] += more_weights[weight_i];
                    weight_i += 1;
                }
            }
        }
    }

    if num_summed >= 1 {
        for i in 0..weights.len() {
            weights[i] /= num_summed as f32;
        }
    }
    return weights
}

fn habit_theory_weights(q: &mut QLearning, agents: &[Agent],
                        agent_dists: &AgentDistanceTable,
                        select_actions: &[Action], agent_i: usize) -> Vec<f32> {
    let agent = &agents[agent_i];
    if agent.habit_theory == 0.0 {
        return vec![0.0; select_actions.len()];
    }

    let mut weights = q_target_weights(q, select_actions, agent);
    let avoid_weights = q_avoid_weights(q, agents, agent_dists, select_actions, agent_i);

    for i in 0..weights.len() {
        weights[i] = (weights[i] + avoid_weights[i]) * agent.habit_theory;
    }

    weights
}

#[derive(Clone)]
struct SearchState {
    map: Rc<WorldMap>,
    forward_predictions: Rc<Vec<Vec<Agent>>>,
    agents: Vec<Agent>,
    agent_i: usize,
    action: Action,
    depth: u32,
}

impl PartialEq for SearchState {
    fn eq(&self, other: &SearchState) -> bool {
        self.agents == other.agents &&
            self.agent_i == other.agent_i &&
            self.action == other.action &&
            self.depth == other.depth
    }
}

impl Eq for SearchState {}

impl Hash for SearchState {
    fn hash<H: Hasher>(&self, state: &mut H) {
        let agent = self.agents[self.agent_i];
        agent.pose.hash(state);
        agent.health.hash(state);
        self.agent_i.hash(state);
        self.action.hash(state);
        self.depth.hash(state);
    }
}

struct SearchExpansion {
    state: SearchState,
    possible_actions: Vec<Action>,
    expansion_i: usize,
}

impl Iterator for SearchExpansion {
    type Item = SearchState;
    fn next(&mut self) -> Option<SearchState> {
        let action = *self.possible_actions.get(self.expansion_i)?;
        let forward_predictions = self.state.forward_predictions.clone();
        let agent_i = self.state.agent_i;
        let depth = self.state.depth + 1;

        // construct new agent set from the already calculated set
        // plus this specific agent from the parent state
        let mut agents = forward_predictions[(depth as usize - 1).min(forward_predictions.len() - 1)].clone();
        agents[agent_i] = self.state.agents[agent_i];
        apply_action(&mut agents[agent_i], action);

        self.expansion_i += 1;

        let map = self.state.map.clone();

        Some(SearchState { map, forward_predictions, agents, agent_i, action, depth })
    }
}

struct FolkSearchTraits;
impl SearchProblemTrait<SearchState> for FolkSearchTraits {
    fn is_goal(&self, node: &SearchNode<SearchState>) -> bool {
        node.state.depth == C.astar_depth

        // let state = &node.state;
        // let agent = state.agents[state.agent_i];
        // let building_agent_i = building_i_to_agent_i(agent.destination_building_i);
        // let building_agent = state.agents[building_agent_i];
        // let dist = obj_dist_1(agent.pose, (agent.width, agent.length),
        //                       building_agent.pose, (building_agent.width, building_agent.length));
        //
        // let building_margin = if agent.kind == AgentKind::Pedestrian
        //                         { BUILDING_PEDESTRIAN_MARGIN } else { BUILDING_VEHICLE_MARGIN };
        //
        // dist <= building_margin
    }

    fn step_cost(&self, node: &SearchNode<SearchState>) -> f32 {
        let state = &node.state;
        let collisions = find_collisions_with(&state.agents, Some(state.agent_i));
        let mut collision_cost = 0.0;
        for c in collisions {
            if c.agent1_i == state.agent_i {
                collision_cost += c.damage1 as f32;
            }
            if c.agent2_i == state.agent_i {
                collision_cost += c.damage2 as f32;
            }
        }
        1.0 + collision_cost
    }

    fn ordering_cost(&self, node: &SearchNode<SearchState>) -> f32 {
        let mut heuristic_cost = 0.0;
        let state = &node.state;
        let agent = state.agents[state.agent_i];
        let building_agent_i = building_i_to_agent_i(agent.destination_building_i);
        let building_agent = state.agents[building_agent_i];
        let dist = obj_dist_1(agent.pose, (agent.width, agent.length),
                              building_agent.pose, (building_agent.width, building_agent.length));
        heuristic_cost += dist as f32 / C.max_forward_vel as f32;

        let building_margin = if agent.kind == AgentKind::Pedestrian
                                { BUILDING_PEDESTRIAN_MARGIN } else { BUILDING_VEHICLE_MARGIN };
        if dist <= building_margin {
            heuristic_cost -= C.search_trip_complete_reward;
        }
        node.path_cost + heuristic_cost
    }

    fn expand_state(&self, node: &SearchNode<SearchState>) -> Box<Iterator<Item = SearchState>> {
        let state = node.state.clone();
        let possible_actions = get_possible_actions(&state.agents[state.agent_i]);
        Box::new(SearchExpansion{state, possible_actions, expansion_i: 0})
    }
}

fn get_search_node_first_action(node: Rc<SearchNode<SearchState>>) -> Action {
    let mut node = node;
    loop {
        if node.depth == 1 {
            return node.state.action;
        }
        if node.parent.is_none() {
            panic!("Expected a parent node!");
        }
        node = node.parent.as_ref().unwrap().clone();
    }
}

fn folk_theory_weights(map: &WorldMap, agents: &[Agent], forward_predictions: Rc<Vec<Vec<Agent>>>,
                       actions: &[Action], agent_i: usize) -> Vec<f32> {
    let mut weights = vec![0.0; actions.len()];
    let agent = &agents[agent_i];
    if agent.folk_theory == 0.0 {
        return weights;
    }

    let map = Rc::new(map.clone());
    let initial_state = SearchState { map, forward_predictions, agents: agents.to_vec(), agent_i,
                                         action: Action::Nothing, depth: 0 };

    let mut p = create_search_problem::<SearchState, PriorityQueue<SearchNode<SearchState>>>
                                        (initial_state, &FolkSearchTraits{}, true, false);
    {
        let result = tree_search(&mut p);
        if let Some(sol) = result {
            let sol_node = sol.leaf;
            // println!("Found solution with ordering cost {:.2} at depth {} after expanding {} nodes", sol_node.ordering_cost, sol_node.depth, sol_node.get_expansion_count());
            let rc_sol = Rc::new(sol_node);

            let action = get_search_node_first_action(rc_sol.clone());
            let action_i = actions.iter().position(|&a| a == action).unwrap();
            weights[action_i] = rc_sol.ordering_cost;

            // let mut node = rc_sol.clone();
            // loop {
            //     println!("Action {:?} at depth {}", node.state.action, node.depth);
            //     if node.parent.is_none() {
            //         break;
            //     }
            //     node = node.parent.as_ref().unwrap().clone();
            // }

            let mut frontier = sol.frontier;
            let mut max_found_val = 0.0; // its a min-heap, so the last value
            loop {
                let node = frontier.remove_next();
                if node.is_none() {
                    break;
                }
                let node = node.unwrap();
                max_found_val = node.ordering_cost;

                if !p.is_goal(&node) {
                    continue;
                }
                let node = Rc::new(node);
                let action = get_search_node_first_action(node.clone());
                let action_i = actions.iter().position(|&a| a == action).unwrap();
                if weights[action_i] == 0.0 || node.ordering_cost < weights[action_i] {
                    weights[action_i] = node.ordering_cost;
                    // println!("Put {} for action {:?}", node.ordering_cost, action);
                }
            }

            // fill in the values empty values
            for i in 0..weights.len() {
                if weights[i] == 0.0 {
                    weights[i] = max_found_val;
                }
                // and invert values since astar minimizes and we want positive to be good
                weights[i] = -weights[i];
            }

            return weights;
        };
    }
    println!("Falied to find solution after expanding {} nodes", p.get_expansion_count());

    weights
}

fn rasterize_poly_line(buff_x0: &mut [i32], buff_x1: &mut [i32], start_x: i32, start_y: i32, end_x: i32, end_y: i32) {
    // Bresenham's Line Drawing, as applied to rasterizing a convex polygon
    // use for rasterizing like with https://stackoverflow.com/questions/10061146/how-to-rasterize-rotated-rectangle-in-2d-by-setpixel
    let mut cx = start_x;
    let mut cy = start_y;

    let dx = (end_x - start_x).abs();
    let dy = (end_y - start_y).abs();

    let sx = (end_x - start_x).signum();
    let sy = (end_y- start_y).signum();

    let mut err = dx - dy;

    loop {
        // still let the alg go through negative cy values, just don't do anything with them
        if cy >= 0 && cy < buff_x0.len() as i32 {
            let cy = cy as usize;
            if sy < 0 {
                buff_x0[cy] = cx;
            } else if sy > 0 {
                buff_x1[cy] = cx;
            } else {
                buff_x0[cy] = if buff_x0[cy] == -1 { cx } else { buff_x0[cy].min(cx) };
                buff_x1[cy] = buff_x1[cy].max(cx);
            }
        }

        if (cx == end_x) && (cy == end_y) {
            return;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err = err - dy;
            cx = cx + sx;
        }
        if e2 < dx {
            err = err + dx;
            cy = cy + sy;
        }
    }
}

fn agent_contained_in_bounds(agent: &Agent, bounds: &BoundingRectI32) -> bool {
    let rect = lookup_bounding_rect(agent.pose, (agent.width, agent.length));
    let ((lx1, ly1), (hx1, hy1)) = rect;
    let &((lx2, ly2), (hx2, hy2)) = bounds;
    lx1 >= lx2 && ly1 >= ly2 && hx1 <= hx2 && hy1 <= hy2
}

fn agent_contained_in_corner(map: &WorldMap, agent: &Agent) -> bool {
    map.corners.iter().any(|c| agent_contained_in_bounds(agent, c))
}

struct MoralSearchTraits;
impl SearchProblemTrait<SearchState> for MoralSearchTraits {
    fn is_goal(&self, node: &SearchNode<SearchState>) -> bool {
        node.state.depth == C.astar_depth
    }

    fn step_cost(&self, node: &SearchNode<SearchState>) -> f32 {
        let state = &node.state;
        let collisions = find_collisions_with(&state.agents, Some(state.agent_i));
        let mut collision_cost = 0.0;
        for c in collisions {
            if c.agent1_i == state.agent_i {
                collision_cost += c.damage1 as f32 * C.damage_penalty;
            }
            if c.agent2_i == state.agent_i {
                collision_cost += c.damage2 as f32 * C.damage_penalty;
            }
        }

        // rasterize agent over the map to see if the locations are all okay
        let agent = &state.agents[state.agent_i];
        let rect = agent_to_rect32(agent);
        let mut buff_x0 = [-1i32; MAP_HEIGHT as usize];
        let mut buff_x1 = [-1i32; MAP_HEIGHT as usize];
        for i in 0..4 {
            let pt1 = rect[i];
            let pt2 = rect[(i + 1) % 4];
            rasterize_poly_line(&mut buff_x0, &mut buff_x1,
                                pt1.0 as i32, pt1.1 as i32, pt2.0 as i32, pt2.1 as i32);
        }

        let mut location_cost = 0.0;

        // which segment(s) of the road is the vehicle on/by?
        let map = &state.map;
        let on_inner_segment = (0..4).map(|i| overlaps_rect(&rect, &map.vehicle_inner_rects[i]))
                                     .collect::<Vec<_>>();
        let on_outer_segment = (0..4).map(|i| overlaps_rect(&rect, &map.vehicle_outer_rects[i]))
                                     .collect::<Vec<_>>();
        //
        // let on_inner_corner = on_inner_segment.iter().filter(|&&b| b).count() > 1;
        let in_corner = agent_contained_in_corner(map, agent);

        // the indexes for the directions on the inner (clockwise) path
        // the counter-clockwise ones will have 3 and 1 swapped and 2 and 0 swapped.
        let agent_dir = if agent.pose.2 < PI_OVER_TWO_INCS / 2 { 3 }
                        else if agent.pose.2 < PI_OVER_TWO_INCS * 3 / 2 { 2 }
                        else if agent.pose.2 < PI_OVER_TWO_INCS * 5 / 2 { 1 }
                        else { 0 };

        let map = &state.map;
        for y in 0..MAP_HEIGHT {
            let x0 = buff_x0[y as usize].max(0) as u32;
            let x1 = buff_x1[y as usize].max(0) as u32;
            for x in x0..x1 {
                let cell = map.grid[(y * MAP_WIDTH + x) as usize];
                if agent.kind == AgentKind::Pedestrian &&
                        cell.intersects(Cell::OUTER_LANE | Cell::INNER_LANE) &&
                        !cell.intersects(Cell::CROSSWALK) {
                   location_cost += C.off_path_penalty
                }
                if agent.kind == AgentKind::Vehicle {
                    if !cell.intersects(Cell::OUTER_LANE | Cell::INNER_LANE) {
                        location_cost += C.off_path_penalty
                    }

                    // INNER_LANE goes clock-wise
                    if cell.intersects(Cell::INNER_LANE) && !on_inner_segment[agent_dir] {
                        location_cost += C.off_lane_penalty;
                    }
                    // we allow _any_ direction at the outerlane corners! (for switching lanes)
                    if !in_corner && cell.intersects(Cell::OUTER_LANE) && !on_outer_segment[(agent_dir + 2) % 4] {
                        location_cost += C.off_lane_penalty;
                    }
                }
            }
        }

        // dismiss very minor misplacements... maybe no the best way to do this...
        // if location_cost <= C.off_lane_penalty && location_cost <= C.off_path_penalty {
        //     location_cost = 0.0;
        // }

        collision_cost + location_cost
    }

    fn ordering_cost(&self, node: &SearchNode<SearchState>) -> f32 {
        node.path_cost
    }

    fn expand_state(&self, node: &SearchNode<SearchState>) -> Box<Iterator<Item = SearchState>> {
        let state = node.state.clone();
        let possible_actions = get_possible_actions(&state.agents[state.agent_i]);
        Box::new(SearchExpansion{state, possible_actions, expansion_i: 0})
    }
}

fn internalization_theory_weights(map: &WorldMap, agents: &[Agent],
                                  forward_predictions: Rc<Vec<Vec<Agent>>>,
                                  actions: &[Action], agent_i: usize) -> Vec<f32> {
    let mut weights = vec![0.0; actions.len()];
    let agent = &agents[agent_i];
    if agent.internalization_theory == 0.0 {
        return weights;
    }

    let map = Rc::new(map.clone());
    for action_i in 0..actions.len() {
        let action = actions[action_i];
        let map = map.clone();
        let forward_predictions = forward_predictions.clone();
        let mut agents = agents.to_vec();

        apply_action(&mut agents[agent_i], action);
        let initial_state = SearchState { map, forward_predictions, agents,
                                          agent_i, action, depth: 0 };

        let mut p = create_search_problem::<SearchState, PriorityQueue<SearchNode<SearchState>>>
                                            (initial_state, &MoralSearchTraits{}, true, false);
        {
            let result = tree_search(&mut p);
            if let Some(sol) = result {
                let sol_node = sol.leaf;
                // println!("Found solution with ordering cost {:.2} at depth {} after expanding {} nodes", sol_node.ordering_cost, sol_node.depth, sol_node.get_expansion_count());
                weights[action_i] = -sol_node.ordering_cost;
                continue;
            };
        }
        println!("Falied to find solution after expanding {} nodes", p.get_expansion_count());
    }
    weights
}

fn choice_restriction_theory_filter(_agents: &[Agent], _agent: &Agent, actions: Vec<(Action, f32)>)
                                    -> Vec<(Action, f32)> {
    actions
}

fn get_possible_actions_by(vel: i32) -> Vec<Action> {
    let mut actions = Vec::new();
    // always possible
    actions.push(Action::TurnLeft);
    actions.push(Action::TurnRight);
    if vel >= 0 && vel < C.max_forward_vel as i32 {
        actions.push(Action::AccelerateForward);
    }
    if vel <= 0 && vel > -(C.max_backward_vel as i32) {
        actions.push(Action::AccelerateBackward);
    }
    if vel != 0 {
        actions.push(Action::Brake);
    }
    actions.push(Action::Continue);

    actions
}

fn get_possible_actions(agent: &Agent) -> Vec<Action> {
    get_possible_actions_by(agent.velocity)
}

fn choose_action(map: &WorldMap, q: &mut QLearning,
                 agents: &mut [Agent], agent_dists: &AgentDistanceTable,
                 forward_predictions: Rc<Vec<Vec<Agent>>>, agent_i: usize) -> Action {
    let actions;
    let choices = {
        let agent = &agents[agent_i];

        if !agent.on_map || agent.kind == AgentKind::Obstacle {
            return Action::Nothing;
        }
        if agent.frozen_steps > 0 {
            return Action::Frozen;
        }

        actions = get_possible_actions(agent);

        let habit_w = habit_theory_weights(q, agents, agent_dists, &actions, agent_i);
        let folk_w = folk_theory_weights(map, agents, forward_predictions.clone(), &actions, agent_i);
        let internal_w = internalization_theory_weights(map, agents, forward_predictions, &actions, agent_i);
        let mut weights = habit_w;
        for i in 0..weights.len() {
            weights[i] += folk_w[i] + internal_w[i];
        }

        let choices = weights.iter().enumerate().map(|(i, &w)| (actions[i], w)).collect::<Vec<_>>();
        choice_restriction_theory_filter(agents, agent, choices)
    };

    let agent = &mut agents[agent_i];

    let mut rng = repeatable_rand(); //rand::thread_rng();

    // let best_choice = choices.iter().fold((Action::Nothing, f32::MIN),
    //                                       |a, &c| if c.1 > a.1 { c } else { a });
    // let mut action_choice = best_choice.0;
    // // if we are a habit-theory agent, prefer exploring a new action first
    // if agent.habit_theory != 0.0 {
    //     let unexplored_actions = choices.iter().filter(|&&(_a, w)| w == 0.0).collect::<Vec<_>>();
    //     if unexplored_actions.len() == 0 {
    //         if rng.gen_weighted_bool(C.explore_one_in_max) {
    //             agent.explore_actions += 1;
    //             if agent.explore_actions <= C.explore_consecutive_limit {
    //                 action_choice = *rng.choose(&actions).unwrap();
    //                 if rng.gen_weighted_bool(10000) {
    //                     println!("Already explored, but choose a random action");
    //                 }
    //             } else {
    //                 agent.explore_actions = 0;
    //                 if rng.gen_weighted_bool(10000) {
    //                     println!("Choose best action to keep from exploring too much in a row");
    //                 }
    //             }
    //         } else {
    //             agent.explore_actions = 0;
    //             if rng.gen_weighted_bool(10000) {
    //                 println!("Choose best action");
    //             }
    //         }
    //     } else {
    //         action_choice = (*rng.choose(&unexplored_actions).unwrap()).0;
    //         if rng.gen_weighted_bool(10000) {
    //             println!("Choice random unexplored action");
    //         }
    //     }
    // }
    // action_choice

    // if we are a habit-theory agent, prefer exploring a new action first
    let mut should_explore = false;
    if agent.habit_theory != 0.0 {
        // let unexplored_actions = choices.iter().filter(|&&(_a, w)| w == 0.0).count() as u32;
        // let n_actions = actions.len() as u32;
        // let explore_one_in = 1 +
        //                      (C.explore_one_in_max - 1) *
        //                      (n_actions - unexplored_actions) / n_actions;
        if q.avoid_empties > C.full_explore_until {
            should_explore = true;
        } else {
            should_explore = rng.gen_weighted_bool(C.explore_one_in_max);
            if should_explore {
                agent.explore_actions += 1;
                if agent.explore_actions > C.explore_consecutive_limit {
                    should_explore = false;
                    agent.explore_actions = 0;
                }
            } else {
                agent.explore_actions = 0;
            }
        }
    }

    // let best_choice = choices.iter().fold((Action::Nothing, f32::MIN),
    //                                       |a, &c| if c.1 > a.1 { c } else { a });
    // if best_choice.1 == 0.0 {
    //     should_explore = true;
    // }

    if should_explore {
        if rng.gen_weighted_bool(10000) {
            println!("Made decision randomly");
        }
        return *rng.choose(&actions).unwrap();
    } else if agent.habit_theory != 0.0 {
        let anneal_start_end = (C.anneal_start - C.anneal_end) as f32;
        let mult_factor = (C.anneal_factor - 1.0) / anneal_start_end;
        let anneal_factor = 1.0 + (q.avoid_empties - C.anneal_end).max(0) as f32 * mult_factor;
        let anneal_factor = 1.0 / anneal_factor;

        // calculate soft-max of weights
        // and use those values as probabilities to choose the action
        let exps = choices.iter().map(|&(_a, w)| (w * anneal_factor).exp()).collect::<Vec<_>>();
        let exp_sum: f32 = exps.iter().sum();
        let soft_max = exps.iter().map(|exp| exp / exp_sum);
        let running_sum = soft_max.fold(Vec::new(), |mut sums, soft| {
            let last_sum = *sums.last().unwrap_or(&0.0);
            sums.push(last_sum + soft);
            sums
        });
        let choice_val = rng.gen_range(0.0, 1.0);
        let choice_i = running_sum.iter().position(|&sum| sum >= choice_val).unwrap();
        if rng.gen_weighted_bool(10000) {
            println!("Made probabilistic decision to use {:?}", choices[choice_i]);
            println!("From choices: {:?}", choices);
        }
        return choices[choice_i].0;
        // return best_choice.0;
    } else {
        // if choices[0].1 == 0.0 && choices[1].1 == -20.0 && choices[2].1 == -45.0 && choices[3].1 == 0.0 && choices[4].1 == -11.0 {
        //     println!("a spot!!! with pose: {:?} and vel: {}", agent.pose, agent.velocity);
        //     thread::sleep(std::time::Duration::from_millis(3000));
        // }

        let best_choice = choices.iter().fold((Action::Nothing, f32::MIN),
                                              |a, &c| if c.1 > a.1 { c } else { a });
        // how many actions have this same value?
        let best_choices = choices.iter().filter(|&&(_a, w)| w == best_choice.1).collect::<Vec<_>>();
        if best_choices.len() == 1 {
            // The do-nothing response (unless imposed by other active agents)
            // means the robot is stuck! In that case, consider other minimally "bad" choices
            if best_choice.0 == Action::Continue && agent.velocity == 0 {
                // panic!("The spot!");
                let alt_choices = choices.iter().filter(
                                    |&&(a, w)| a != Action::Continue && w >= C.moral_forgiveness)
                                    .collect::<Vec<_>>();
                if alt_choices.len() > 0 {
                    let choice = **rng.choose(&alt_choices).unwrap();
                    if rng.gen_weighted_bool(1000) {
                        println!("Chose an alternative folk/internaliztion theory choice to avoid doing nothing: {:?}", best_choice.0);
                        println!("From choices: {:?}", choices);
                    }
                    return choice.0;
                }
            }
            if rng.gen_weighted_bool(1000) {
                println!("Made deterministic folk/internaliztion theory choice to use {:?}", best_choice.0);
                println!("From choices: {:?}", choices);
            }
            return best_choice.0;
        }
        // choose an equivalent action randomly
        let choice = **rng.choose(&best_choices).unwrap();
        if rng.gen_weighted_bool(1000) {
            println!("Randomly chose a best-valued folk/internaliztion theory choice to use {:?}", best_choice.0);
            println!("From choices: {:?}", choices);
        }
        return choice.0;
    }
}

fn apply_action(agent: &mut Agent, action: Action) {
    match action {
        Action::TurnLeft => {
            let rot_pt1 = rotate_pt((agent.width as f32 / 2.0,
                                     agent.length as f32 / 2.0),
                                     -(agent.pose.2 as i32));
            agent.pose.2 = (agent.pose.2 + 1) % TWO_PI_INCS;
            let rot_pt2 = rotate_pt((agent.width as f32 / 2.0,
                                     agent.length as f32 / 2.0),
                                     -(agent.pose.2 as i32));
            agent.pose.0 = (agent.pose.0 as f32 + rot_pt1.0 - rot_pt2.0 + 0.5).max(0.0) as u32;
            agent.pose.1 = (agent.pose.1 as f32 + rot_pt1.1 - rot_pt2.1 + 0.5).max(0.0) as u32;
        },
        Action::TurnRight => {
            let rot_pt1 = rotate_pt((agent.width as f32 / 2.0,
                                     agent.length as f32 / 2.0),
                                     -(agent.pose.2 as i32));
            agent.pose.2 = (agent.pose.2 + TWO_PI_INCS - 1) % TWO_PI_INCS;
            let rot_pt2 = rotate_pt((agent.width as f32 / 2.0,
                                     agent.length as f32 / 2.0),
                                     -(agent.pose.2 as i32));
            agent.pose.0 = (agent.pose.0 as f32 + rot_pt1.0 - rot_pt2.0 + 0.5).max(0.0) as u32;
            agent.pose.1 = (agent.pose.1 as f32 + rot_pt1.1 - rot_pt2.1 + 0.5).max(0.0) as u32;
        },
        Action::AccelerateForward => {
            agent.velocity = (agent.velocity + C.forward_accel as i32)
                             .min(C.max_forward_vel as i32);
        },
        Action::AccelerateBackward => {
            agent.velocity = (agent.velocity - C.backward_accel as i32)
                             .max(-(C.max_backward_vel as i32));
        },
        Action::Brake => {
            if agent.velocity.abs() <= C.brake_power as i32 {
                agent.velocity = 0;
            } else {
                agent.velocity += if agent.velocity > 0 { -(C.brake_power as i32) } else { C.brake_power as i32 };
            }
        },
        Action::Continue => {
        },
        Action::Frozen => {
            if agent.frozen_steps > 0 {
                agent.frozen_steps -= 1;
            }
            return;
        },
        Action::Nothing => {
            return;
        }
    }
    let (sin_t, cos_t) = fast_sin_cos(agent.pose.2 as i32);

    let new_x = agent.pose.0 as f32 - agent.velocity as f32 * sin_t;
    let new_x = (new_x + 0.5).max(0.0).min(MAP_WIDTH as f32);
    agent.pose.0 = new_x as u32;

    let new_y = agent.pose.1 as f32 - agent.velocity as f32 * cos_t;
    let new_y = (new_y + 0.5).max(0.0).min(MAP_HEIGHT as f32);
    agent.pose.1 = new_y as u32;
}

fn simple_forward_timestep(agents: &[Agent]) -> Vec<Agent> {
    let mut new_agents = Vec::with_capacity(agents.len());
    for i in 0..agents.len() {
        let mut new_agent = agents[i];
        if new_agent.on_map && new_agent.frozen_steps == 0 && new_agent.kind != AgentKind::Obstacle {
            apply_action(&mut new_agent, Action::Continue);
        }
        new_agents.push(new_agent);
    }
    new_agents
}

fn calc_forward_predictions(agents: &[Agent]) -> Vec<Vec<Agent>> {
    if C.use_single_agent_type &&
            C.folk_theory == 0.0 &&
            C.internalization_theory == 0.0 &&
            !C.choice_restriction_theory {
        return Vec::new();
    }
    let mut forward_predictions: Vec<Vec<Agent>> = Vec::new();
    for i in 0..C.astar_depth {
        let next_agents;
        if i == 0 {
            next_agents = simple_forward_timestep(agents);
        } else {
            next_agents = simple_forward_timestep(forward_predictions.last().unwrap());
        }
        forward_predictions.push(next_agents);
    }
    forward_predictions
}

fn update_agents(map: &WorldMap, q: &mut QLearning,
                 agents: &mut [Agent], agent_dists: &AgentDistanceTable) -> Vec<Action> {
    let forward_predictions = Rc::new(calc_forward_predictions(agents));
    let actions = (0..agents.len()).map(|i|
                            choose_action(map, q, agents, agent_dists,
                                          forward_predictions.clone(), i)).collect::<Vec<_>>();
    for (i, &action) in actions.iter().enumerate() {
        apply_action(&mut agents[i], action);

        if action == Action::Frozen && agents[i].frozen_steps == 0 {
            if agents[i].health == 0 {
                agents[i].on_map = false;
            } else {
                clear_from_collision(agents, i);
            }
        }
    }
    actions
}

#[derive(Debug, Clone)]
struct Collision {
    agent1_i: usize,
    agent2_i: usize,
    damage1: i32,
    damage2: i32,
}

fn bounds_intersect(a: BoundingRectI32, b: BoundingRectI32) -> bool {
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
    let (sin_t, cos_t) = fast_sin_cos(agent.pose.2 as i32);
    (agent.velocity as f32 * sin_t, agent.velocity as f32 * cos_t)
}

// optionally only looks for collisions where a specified agent index is one of the two agents
fn find_collisions_with(agents: &[Agent], single_agent: Option<usize>) -> Vec<Collision> {
    let mut collisions = Vec::new();

    let bounding_rects = agents.iter().map(|a|
                                        lookup_bounding_rect(a.pose, (a.width, a.length)));
    let bounding_rects = bounding_rects.collect::<Vec<_>>();
    let agent_rects = agents.iter().map(|a| agent_to_rect32(a)).collect::<Vec<_>>();

    let agent1_range = if single_agent.is_none() {
        0..(agents.len() - 1)
    } else {
        0..agents.len()
    };

    for agent1_i in agent1_range {
        let agent1 = agents[agent1_i];
        if !agent1.on_map {
            continue;
        }

        let agent2_range;
        if single_agent.is_none() {
            agent2_range = (agent1_i + 1)..agents.len();
        } else {
            let agent_i = single_agent.unwrap();
            if agent1_i == agent_i {
                continue;
            }
            agent2_range = agent_i..(agent_i + 1);
        }

        let bounds1 = bounding_rects[agent1_i];
        for agent2_i in agent2_range {
            // if !single_agent.is_none() && (single_agent.unwrap() != agent1_i &&
            //                                single_agent.unwrap() != agent2_i) {
            //     continue; // not relevant for our current search
            // }
            let agent2 = agents[agent2_i];
            if !agent2.on_map {
                continue;
            }
            if (agent1.frozen_steps > 0 || agent1.kind == AgentKind::Obstacle) &&
               (agent2.frozen_steps > 0 || agent2.kind == AgentKind::Obstacle) {
                // old collision
                continue;
            }

            let bounds2 = bounding_rects[agent2_i];
            // first perform lowest-res collision test
            if !bounds_intersect(bounds1, bounds2) {
                continue;
            }

            if !overlaps_rect(&agent_rects[agent1_i], &agent_rects[agent2_i]) {
                continue;
            }

            // if single_agent.is_none() {
            //     print!("Collision! ");
            // }

            // calculate severity of the collision based on the relative angle of the agents,
            // penetration depth, relative velocity, and agent type... roughly!

            let mut intensity = 0.0;
            for &(agent_a_i, agent_b_i) in [(agent1_i, agent2_i), (agent2_i, agent1_i)].iter() {
                let agent_a = agents[agent_a_i];
                let agent_b = agents[agent_b_i];

                // project velocities and rects onto agent a's direction of motion
                let vel_a = get_velocity_vec(&agent_a);
                let vel_b = get_velocity_vec(&agent_b);

                let theta = agent_a.pose.2 as i32;

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
                // limit to half-way through the "body" of the agent in this dimension.
                let depth = depth.min((max_a - min_a) / 2.0);

                let additional_intensity = depth.max(1.0) * rel_vel.abs().max(1.0);
                intensity += additional_intensity;
                // if single_agent.is_none() {
                //     print!("depth {} rel_vel {} for {} ;", depth, rel_vel, additional_intensity);
                // }
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
            // if single_agent.is_none() {
            //     println!(" {:?} took {:?} and {:?} took {:?}",
            //              agent1.kind, damage1, agent2.kind, damage2);
            // }
            collisions.push(collision);
        }
    }

    collisions
}

fn find_collisions(agents: &[Agent]) -> Vec<Collision> {
    find_collisions_with(agents, None)
}

fn clear_from_collision(agents: &mut [Agent], agent_i: usize) {
    // try to move the target agent as little as possible to clear it from all others
    // following a collision
    let pose = agents[agent_i].pose;
    for &(dx, dy) in [(1, 1), (-1, -1), (1, -1), (-1, 1),
                     (0, 2), (0, -2), (2, 0), (-2, 0),
                     (2, 2), (-2, -2), (2, -2), (-2, 2),
                     (0, 3), (0, -3), (3, 0), (-3, 0),
                     (3, 3), (-3, -3), (3, -3), (-3, 3),
                     (0, 4), (0, -4), (4, 0), (-4, 0),
                     (4, 4), (-4, -4), (4, -4), (-4, 4)].iter() {
    // }
    // for dy in [0, 1, -1, 2, -2, 3, -3, 4, -4].iter() {
    //     for dx in [0, 1, -1, 2, -2, 3, -3, 4, -4].iter() {
        let mut new_pose = pose;
        new_pose.0 = (new_pose.0 as isize + dx).max(1) as u32;
        new_pose.0 = new_pose.0.min(MAP_WIDTH - 1);
        new_pose.1 = (new_pose.1 as isize + dy).max(1) as u32;
        new_pose.1 = new_pose.1.min(MAP_HEIGHT - 1);
        agents[agent_i].pose = new_pose;
        if find_collisions_with(agents, Some(agent_i)).len() == 0 {
            // println!("Freed collision by moving agent {} by ({}, {})", agent_i, dx, dy);
            return;
        }
        // }
    }
    // could not find a way to save the person. Count them as dead!
    // println!("Could not free agent {}, counting it dead", agent_i);
    agents[agent_i].pose = pose;
    agents[agent_i].health = 0;
    agents[agent_i].velocity = 0;
    agents[agent_i].on_map = false;
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

fn evolve_new_agent(agents: &[Agent], kind: AgentKind) -> Agent {
    let alive = agents.iter().filter(|a| a.health > 0 &&
                                         a.kind == kind).collect::<Vec<_>>();
    let mut rng = repeatable_rand(); //rand::thread_rng();
    let mut new_agent = **rng.choose(&alive).unwrap();
    new_agent.health = C.total_health as i32;
    new_agent.on_map = false;
    new_agent.velocity = 0;
    new_agent
}

fn find_dead_agents(agents: &[Agent]) -> Vec<usize> {
    let mut replaced = Vec::new();
    // replace agents that have died
    for i in 0..agents.len() {
        let a = &agents[i];
        if !a.on_map && a.health == 0 && a.kind != AgentKind::Obstacle {
            replaced.push(i);
        }
    }
    replaced
}

// returns number of agents replaced
fn replace_dead_agents(agents: &mut [Agent], dead: &Vec<usize>) {
    for &i in dead {
        agents[i] = evolve_new_agent(agents, agents[i].kind);
    }
}

// returns the number of newly completed trips
fn evaluate_trips(map: &WorldMap, agents: &mut [Agent], agent_dists: &AgentDistanceTable) -> Vec<usize> {
    let mut trips_completed = Vec::new();

    for (i, agent) in agents.iter_mut().enumerate() {
        if !agent.on_map || agent.frozen_steps > 0 || agent.kind == AgentKind::Obstacle {
            continue;
        }
        let building = map.buildings[agent.destination_building_i as usize];
        let dist = get_agent_1_dist(agent_dists, i, building.agent_i);
        let building_margin = if agent.kind == AgentKind::Pedestrian
                                { BUILDING_PEDESTRIAN_MARGIN } else { BUILDING_VEHICLE_MARGIN };
        if dist <= building_margin {
            agent.on_map = false;
            agent.health = (agent.health + C.heal_reward).min(C.total_health as i32);
            agent.velocity = 0;
            trips_completed.push(i);
        }
    }

    trips_completed
}

fn update_q_target_table(q: &mut QLearning, q_targets: &Vec<QStateTarget>,
                         actions: &[Action], new_qs: &[Option<f32>]) {
    let all_actions = Action::normal_actions();
    for (i, &new_q) in new_qs.iter().enumerate() {
        let new_q = if let Some(new_q) = new_q { new_q } else { continue };
        let action_i = all_actions.iter().position(|&a| a == actions[i]);
        let action_i = if let Some(action_i) = action_i { action_i } else { continue };

        if !q.target_values.contains_key(&q_targets[i]) {
            q.target_values.insert(q_targets[i], vec![0.0; all_actions.len()]);
        }
        let mut q_target = q.target_values.get_mut(&q_targets[i]).unwrap();
        let old_val = q_target[action_i];
        let new_val = (1.0 - C.q_learn_rate) * old_val + C.q_learn_rate * new_q;
        q_target[action_i] = new_val;
    }
}

fn update_q_avoid_tables(q: &mut QLearning, q_avoids: &Vec<Vec<(usize, QStateAvoid)>>,
                         collisions: &[Collision], actions: &[Action], base_qs: &[Option<f32>]) {
    let all_actions = Action::normal_actions();
    for (i, &base_q) in base_qs.iter().enumerate() {
        let base_q = if let Some(base_q) = base_q { base_q } else { continue };
        let action_i = all_actions.iter().position(|&a| a == actions[i]);
        let action_i = if let Some(action_i) = action_i { action_i } else { continue };

        let q_avoids = &q_avoids[i];
        for &(other_i, q_avoid) in q_avoids {
            let mut new_q = base_q;
            for c in collisions.iter() {
                if c.agent1_i == i && c.agent2_i == other_i {
                    new_q -= C.damage_coef * c.damage1 as f32;
                }
                if c.agent2_i == i && c.agent1_i == other_i {
                    new_q -= C.damage_coef * c.damage2 as f32;
                }
            }

            if !q.avoid_values.contains_key(&q_avoid) {
                q.avoid_values.insert(q_avoid, vec![0.0; all_actions.len()]);
            }
            let mut q_avoid_val = q.avoid_values.get_mut(&q_avoid).unwrap();
            let old_val = q_avoid_val[action_i];
            let new_val = (1.0 - C.q_learn_rate) * old_val + C.q_learn_rate * new_q;
            q_avoid_val[action_i] = new_val;
        }
    }
}

fn calc_q_avoids_states(agents: &[Agent], agent_dists: &AgentDistanceTable) -> Vec<Vec<(usize, QStateAvoid)>> {
    let mut q_avoids = Vec::new();
    for i in 0..agents.len() {
        let mut agent_avoids = Vec::new();
        let agent = &agents[i];
        if !agent.on_map {
            q_avoids.push(agent_avoids);
            continue;
        }
        for j in 0..agents.len() {
            let dist = get_agent_1_dist(&agent_dists, i, j);
            if dist <= C.state_avoid_dist {
                let q_avoid = qstate_avoid_for_agent(agent, &agents[j]);
                agent_avoids.push((j, q_avoid));
            }
        }
        q_avoids.push(agent_avoids);
    }
    q_avoids
}

fn calc_best_q_value(q: &mut QLearning, for_target: bool, agents: &[Agent],
                            agent_dists: &AgentDistanceTable, agent_i: usize) -> Option<f32> {
    let agent = &agents[agent_i];
    if !agent.on_map || agent.kind == AgentKind::Obstacle {
        return None;
    }
    if agent.health == 0 {
        // will not continue from this state, so cannot have reward from acting in it
        return Some(0.0)
    }
    let actions = get_possible_actions(agent);
    let weights = if for_target {
        q_target_weights(q, &actions, agent)
    } else {
        q_avoid_weights(q, agents, agent_dists, &actions, agent_i)
    };
    let max_w = weights.iter().fold(f32::MIN, |a, &b| a.max(b));
    Some(max_w)
}

fn calc_best_q_target_value(q: &mut QLearning, agents: &[Agent],
                            agent_dists: &AgentDistanceTable, agent_i: usize) -> Option<f32> {
    calc_best_q_value(q, true, agents, agent_dists, agent_i)
}

fn calc_best_q_avoid_value(q: &mut QLearning, agents: &[Agent],
                            agent_dists: &AgentDistanceTable, agent_i: usize) -> Option<f32> {
    calc_best_q_value(q, false, agents, agent_dists, agent_i)
}

fn add_q_reward(new_qs: &mut Vec<Option<f32>>, agent_is: &[usize], score: f32) {
    for &i in agent_is {
        if let Some(val) = new_qs[i] {
            new_qs[i] = Some(val + score);
        }
    }
}

struct AgentDistanceTable {
    v: Vec<u32>,
    n: usize,
}

fn compute_agent_1_dists(agents: &[Agent]) -> AgentDistanceTable {
    let n = agents.len();
    // default to max value so agents are not considered
    // colliding with themselves or with off-map agents.
    let mut v = vec![u32::max_value(); n.pow(2)];
    for i in 1..agents.len() {
        let a1 = &agents[i];
        for j in 0..i {
            let a2 = &agents[j];
            if !a1.on_map || !a2.on_map {
                continue;
            }
            let dist = obj_dist_1(a1.pose, (a1.width, a1.length), a2.pose, (a2.width, a2.length));
            v[i * n + j] = dist;
            v[j * n + i] = dist;
        }
    }
    AgentDistanceTable { v, n }
}

fn get_agent_1_dist(dists: &AgentDistanceTable, agent1_i: usize, agent2_i: usize) -> u32 {
    dists.v[agent1_i * dists.n + agent2_i]
}

fn run_timestep(map: &WorldMap, q: &mut QLearning, agents: &mut [Agent], stats: &mut WorldStats) {
    let agent_dists = compute_agent_1_dists(&agents);

    let prior_q_targets = agents.iter().map(|a| qstate_target_for_agent(a)).collect::<Vec<_>>();
    let prior_q_avoids = calc_q_avoids_states(agents, &agent_dists); // here

    let habit_ws = (0..agents.len()).map(|i| {
        if q.avoid_empties <= C.debug_choices_after {
            let possible_actions = get_possible_actions(&agents[i]);
            habit_theory_weights(q, agents, &agent_dists, &possible_actions, i)
        } else {
            Vec::new()
        }
    }).collect::<Vec<_>>();

    let actions = update_agents(map, q, agents, &agent_dists); // here
    let collisions = find_collisions(agents);
    stats.collisions += collisions.len() as u32;

    resolve_collisions(agents, &collisions);

    let agent_dists = compute_agent_1_dists(&agents);
    let mut new_target_qs = (0..agents.len()).map(|i| Some(C.q_discount *
                                    calc_best_q_target_value(q, agents, &agent_dists, i)?))
                            .collect::<Vec<_>>();
    let base_avoid_qs = (0..agents.len()).map(|i| Some(C.q_discount *
                                    calc_best_q_avoid_value(q, agents, &agent_dists, i)?))
                            .collect::<Vec<_>>();

    //  collision data is applied more precisely for the q-avoid tables, so it is done separately
    for c in collisions.iter() {
        if let Some(val) = new_target_qs[c.agent1_i] {
            new_target_qs[c.agent1_i] = Some(val - C.damage_coef * c.damage1 as f32);
        }
        if let Some(val) = new_target_qs[c.agent2_i] {
            new_target_qs[c.agent2_i] = Some(val - C.damage_coef * c.damage2 as f32);
        }
    }

    if collisions.len() > 0 && q.avoid_empties <= C.debug_choices_after {
        let coll_i = collisions[0].agent1_i;
        println!("Habit weights w/ vel {} that lead to choose action {:?} with collision result: {:?}",
                 prior_q_targets[coll_i].vel, actions[coll_i], habit_ws[coll_i]);
    }

    let trips_completed = evaluate_trips(map, agents, &agent_dists); // here
    stats.trips_completed += trips_completed.len() as u32;
    add_q_reward(&mut new_target_qs, &trips_completed, C.trip_reward);

    let dead = find_dead_agents(agents);
    stats.deaths += dead.len() as u32;
    // add_q_reward(&mut new_target_qs, &dead, -100.0);
    // add_q_reward(&mut base_avoid_qs, &dead, -100.0);

    // constant for the timestep
    let active_agents = (0..agents.len()).filter(|&i|
                                agents[i].on_map && agents[i].kind != AgentKind::Obstacle)
                                .collect::<Vec<_>>();
    add_q_reward(&mut new_target_qs, &active_agents, -1.0);

    update_q_target_table(q, &prior_q_targets, &actions, &new_target_qs);
    update_q_avoid_tables(q, &prior_q_avoids, &collisions, &actions, &base_avoid_qs);

    replace_dead_agents(agents, &dead);
    replenish_pedestrians(map, agents);
    replenish_vehicles(map, agents);
}

#[get("/")]
fn index(state: State<WorldState>) -> Template {
    Template::render("index", &state.lock().unwrap().map)
}

#[get("/update")]
fn get_update(state: State<WorldState>) -> Json<Value> {
    Json(json!(&*state.lock().unwrap()))
}

#[get("/resources/<file..>")]
fn resources(file: PathBuf) -> Option<NamedFile> {
    NamedFile::open(Path::new("resources/").join(file)).ok()
}

fn q_diagnostic(q: &QLearning) {
    let mut f = match fs::File::create("q_diag.csv") {
        Ok(f) => f,
        Err(e) => panic!("file error: {}", e),
    };

    for y in 0..MAP_HEIGHT {
        for x in 0..MAP_WIDTH {
            let mut best = f32::MIN;
            let mut worst = f32::MAX;
            let mut accum = 0.0;
            let mut accum_n = 0;
            for vel in -3..4 {
                let possible_actions = get_possible_actions_by(vel);

                for theta in 0..TWO_PI_INCS {
                    let q_targ = QStateTarget {x, y, theta, vel, destination_building_i: 0, is_pedestrian: false };
                    let vals = q.target_values.get(&q_targ);
                    if vals.is_none() {
                        continue;
                    }

                    let vals = vals.unwrap();
                    for (i, &val) in vals.iter().enumerate() {
                        let val_action = Action::normal_actions()[i];
                        if possible_actions.iter().find(|&&a| a == val_action).is_none() {
                            continue;
                        }

                        accum += val;
                        accum_n += 1;
                        if val > best {
                            best = val;
                        }
                        if val < worst {
                            worst = val;
                        }
                    }
                }
            }
            if accum_n == 0 {
                write!(f, "{}, ", f32::MIN).unwrap();
            } else {
                write!(f, "{}, ", 0.0 * worst + 0.0 * accum / accum_n as f32 + 1.0 * best).unwrap();
            }
        }
        writeln!(f, "").unwrap();
    }
}

fn q_avoid_count_empties(q: &QLearning) -> u32 {
    let mut empties = 0;
    let mut action_empties = [0; 6];
    let normal_actions = Action::normal_actions();

    let avoid_dist = C.state_avoid_dist as i32; // -1 because we use less than this dist.
    for other_y in -avoid_dist..(avoid_dist + 1) {
        for other_x in -avoid_dist..(avoid_dist + 1) {
            if other_y.abs() + other_x.abs() > avoid_dist {
                continue;
            }
            for &vel in [-1, 0, 1].iter() {
                let possible_actions = get_possible_actions_by(vel);
                let q_avoid = QStateAvoid {vel, other_x, other_y, is_pedestrian: false };
                let vals = q.avoid_values.get(&q_avoid);
                if vals.is_none() {
                    empties += possible_actions.len() as u32;
                    for &action in possible_actions.iter() {
                        let action_i = normal_actions.iter().position(|&a| a == action).unwrap();
                        action_empties[action_i] += 1;
                    }
                    continue;
                }
                let vals = vals.unwrap();
                for (i, &val) in vals.iter().enumerate() {
                    let val_action = normal_actions[i];
                    if possible_actions.iter().find(|&&a| a == val_action).is_none() {
                        continue;
                    }

                    if val == 0.0 {
                        empties += 1;
                        action_empties[i] += 1;
                    }
                }
            }
        }
    }
    println!("Currently at {} empties! by action: {:?}", empties, action_empties);
    empties
}

fn q_avoid_diagnostic(q: &QLearning) {
    let mut f_x = match fs::File::create("q_avoid_diag_x.csv") {
        Ok(f_x) => f_x,
        Err(e) => panic!("file error: {}", e),
    };
    let mut f_y = match fs::File::create("q_avoid_diag_y.csv") {
        Ok(f_y) => f_y,
        Err(e) => panic!("file error: {}", e),
    };

    let mut empties = 0;

    let avoid_dist = C.state_avoid_dist as i32; // -1 because we use less than this dist.
    for other_y in -avoid_dist..(avoid_dist + 1) {
        for other_x in -avoid_dist..(avoid_dist + 1) {
            if other_y.abs() + other_x.abs() > avoid_dist {
                write!(f_x, "{}, ", f32::NAN).unwrap();
                write!(f_y, "{}, ", f32::NAN).unwrap();
                continue;
            }
            let mut best = f32::MIN;
            let mut best_action_i = 0;
            let mut best_vel = 0;
            let mut acc_val = 0.0;
            let mut acc_count = 0;
            for &vel in [-1].iter() {//-3..4 {
                let possible_actions = get_possible_actions_by(vel);
                let q_avoid = QStateAvoid {vel, other_x, other_y, is_pedestrian: false };
                let vals = q.avoid_values.get(&q_avoid);
                if vals.is_none() {
                    empties += possible_actions.len();
                    continue;
                }
                let vals = vals.unwrap();
                for (i, &val) in vals.iter().enumerate() {
                    let val_action = Action::normal_actions()[i];
                    if possible_actions.iter().find(|&&a| a == val_action).is_none() {
                        continue;
                    }

                    if val == 0.0 {
                        empties += 1;
                        continue;
                    }
                    // if val_action == Action::TurnRight {
                    //     continue;
                    // }

                    if val > best {
                        best = val;
                        best_action_i = i;
                        best_vel = vel;
                        acc_val += val.abs();
                        acc_count += 1;
                    }
                }
                // if best != f32::MIN {
                //     println!("TL: {} TR: {} best: {} best_i: {}", vals[0], vals[1], best, best_action_i);
                // }
            }

            let best_action = Action::normal_actions()[best_action_i];

            let new_direction = match best_action {
                Action::TurnLeft => if best_vel < 0 { consts::PI + THETA_PER_INC} else { THETA_PER_INC },
                Action::TurnRight => if best_vel < 0 { consts::PI - THETA_PER_INC} else { 2.0 * consts::PI - THETA_PER_INC },
                Action::AccelerateForward => 0.0,
                Action::AccelerateBackward => consts::PI,
                Action::Brake => if best_vel < 0 { 0.0 } else { consts::PI },
                Action::Continue => if best_vel < 0 { consts::PI } else { 0.0 },
                Action::Frozen => panic!("Action::Frozen invalid in q avoid diagnostic"),
                Action::Nothing => panic!("Action::Nothing invalid in q avoid diagnostic"),
            };

            let x_part = -new_direction.sin() * acc_val / acc_count as f32;
            let y_part = new_direction.cos() * acc_val / acc_count as f32;

            write!(f_x, "{}, ", x_part).unwrap();
            write!(f_y, "{}, ", y_part).unwrap();
        }
        writeln!(f_x, "").unwrap();
        writeln!(f_y, "").unwrap();
    }

    println!("{} empties at vel=-1/-2/-3 remain in q_avoid table", empties);
}

fn main() {
    let mut map = create_map();
    setup_map_paths(&mut map);

    let mut agents = create_agents(&mut map);
    setup_agents(&map, &mut agents);

    let state = WorldStateInner {map, agents: agents,
                                      stats: WorldStats::default()};
    let state = Arc::new(Mutex::new(state));

    let (io_sender, io_receiver) = channel();
    thread::spawn(move || {
        loop {
            // send a unit () for each line we get
            let mut input = String::new();
            match io::stdin().read_line(&mut input) {
                Ok(_) => {
                    io_sender.send(()).unwrap();
                }
                Err(_) => {},
            }
        }
    });

    let thread_state = state.clone();
    let mut q = QLearning::default();
    q.avoid_empties = q_avoid_count_empties(&q);

    let mut iter = 0;
    let mut time_step = 0;
    let mut run_slow = false;
    let mut simulation_running = true;
    let mut counter_reset_finished = false;
    thread::spawn(move || {
        loop {
            if io_receiver.try_recv().is_ok() {
                run_slow = !run_slow;
                q_diagnostic(&q);
                q_avoid_diagnostic(&q);
                println!("On timestep {}k", time_step / 1000);
            }
            if !simulation_running {
                thread::sleep(std::time::Duration::from_millis(C.slow_step_ms as u64));
                continue;
            }
            if run_slow {
                thread::sleep(std::time::Duration::from_millis(C.slow_step_ms as u64));
            } else if (iter % C.fast_steps_per_update) == 0 {
                thread::sleep(std::time::Duration::from_millis(1));
                iter = 0;
            }
            iter += 1;
            time_step += 1;

            let thread_state = &mut *thread_state.lock().unwrap();
            run_timestep(&thread_state.map, &mut q,
                         &mut thread_state.agents, &mut thread_state.stats);
            thread_state.stats.timesteps = time_step;

            if (time_step % C.fast_steps_update_q_empties) == 0 {
                q.avoid_empties = q_avoid_count_empties(&q);
            }

            if time_step >= C.reset_counters_at_timestep && !counter_reset_finished {
                counter_reset_finished = true;
                thread_state.stats.deaths = 0;
                thread_state.stats.collisions = 0;
                thread_state.stats.trips_completed = 0;
            }

            if time_step >= C.timestep_limit {
                println!("Completed {} timesteps. Stopping simulation.", time_step);
                simulation_running = false;
            }
        }
    });

    rocket::ignite()
        .manage(state.clone())
        .mount("/", routes![index, get_update, resources])
        .attach(Template::fairing())
        .launch();
}

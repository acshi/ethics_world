use super::*;
use std::collections::HashSet;
use std::hash::{Hash, Hasher};

#[derive(PartialEq, Clone, Debug)]
struct Pose {
    x: usize,
    y: usize,
    theta: f32,
}

impl Eq for Pose {}

impl Hash for Pose {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
        self.theta.to_bits().hash(state);
    }
}

#[test]
fn test_initially_unoccupied() {
    let mut map = create_map();
    setup_map_paths(&mut map);

    let agents = create_agents();

    let path_width = map.vert_sidewalk_width.max(map.horiz_sidewalk_width);
    let on_outer = calc_occupied_perim_map(&agents, &map.vehicle_outer_pts, path_width, false);
    let on_inner = calc_occupied_perim_map(&agents, &map.vehicle_inner_pts, path_width, true);

    assert!(on_outer.iter().all(|&b| !b));
    assert!(on_inner.iter().all(|&b| !b));
}

#[test]
fn test_spawning_unique() {
    for i in 0..100 {
        println!("\nAttempt #: {}", i);

        let mut map = create_map();
        setup_map_paths(&mut map);

        let mut agents = create_agents();
        setup_agents(&map, &mut agents);

        let all_agents = agents.pedestrians.iter().chain(agents.vehicles.iter()).filter(|a| a.on_map);
        let mut all_poses = all_agents.map(|a| Pose{x: a.pose.0, y: a.pose.1, theta: a.pose.2}).collect::<Vec<_>>();
        let all_pose_len = all_poses.len();
        let set_poses = all_poses.drain(..).collect::<HashSet<_>>(); // remove duplicate values

        assert_eq!(all_pose_len, set_poses.len());
    }
}

#[test]
fn test_spawn_pedestrian() {
    for i in 0..100 {
        println!("\nAttempt #: {}", i);

        let mut map = create_map();
        setup_map_paths(&mut map);
        let mut agents = create_agents();

        let agent_i = draw_off_map_agent_i(&mut agents.pedestrians);
        if agent_i.is_none() {
            continue; // no agents left to draw
        }
        let agent_i = agent_i.unwrap();

        let path_width = map.vert_sidewalk_width.max(map.horiz_sidewalk_width);
        let on_outer = calc_occupied_perim_map(&agents, &map.pedestrian_outer_pts, path_width, false);
        let on_inner = calc_occupied_perim_map(&agents, &map.pedestrian_inner_pts, path_width, true);

        let pose = spawn_on_two_perims(&map, &map.buildings, BUILDING_PEDESTRIAN_MARGIN,
                                       (PEDESTRIAN_SIZE, PEDESTRIAN_SIZE),
                                       &map.pedestrian_outer_pts, &map.pedestrian_inner_pts,
                                       &on_outer, &on_inner, SPAWN_MARGIN);

        println!("Inner points: {:?}", map.pedestrian_inner_pts);

        if let Some(pose) = pose {
            let mut agent = &mut agents.pedestrians[agent_i];
            println!("Drew pose: {:?} and pedestrian: {:?}", pose, agent);
            agent.on_map = true;
            agent.pose = pose;

            let (width, length) = (agent.width as f32, agent.length as f32);
            let (x, y, theta) = (agent.pose.0 as f32, agent.pose.1 as f32, agent.pose.2);
            let rot_width = rotate_pt((width, 0.0), -theta);
            let rot_length = rotate_pt((0.0, length), -theta);
            let agent_rect = [(x, y),
                             (x + rot_width.0, y + rot_width.1),
                             (x + rot_width.0 + rot_length.0, y + rot_width.1 + rot_length.1),
                             (x + rot_length.0, y + rot_length.1)];
            println!("Examining agent with rect: {:?}", agent_rect);
        }

        let on_outer = calc_occupied_perim_map(&agents, &map.pedestrian_outer_pts, path_width, false);

        // for _ in 0..10 {
        //     println!("*** Looking at inner now! ***");
        // }

        let on_inner = calc_occupied_perim_map(&agents, &map.pedestrian_inner_pts, path_width, true);

        for (i, &b) in on_inner.iter().enumerate() {
            if b {
                println!("{}", i);
            }
        }
        //
        // for (i, &b) in on_outer.iter().enumerate() {
        //     if b {
        //         println!("Outer: {}", i);
        //     }
        // }

        let occ_count_outer = on_outer.iter().filter(|&&b| b).count();
        let occ_count_inner = on_inner.iter().filter(|&&b| b).count();

        // can only be on one of the two lanes/paths
        assert!(occ_count_inner == 0 || occ_count_outer == 0);
        assert_eq!(occ_count_outer + occ_count_inner, PEDESTRIAN_SIZE);
    }
}

#[test]
fn test_spawn_vehicle() {
    for i in 0..4 {
        println!("\nAttempt #: {}", i);

        let mut map = create_map();
        setup_map_paths(&mut map);
        let mut agents = create_agents();

        // replenish_pedestrians(&map, &mut agents);

        for _ in 0..VEHICLE_N {
            let agent_i = draw_off_map_agent_i(&mut agents.vehicles);
            if agent_i.is_none() {
                continue; // no agents left to draw
            }
            let agent_i = agent_i.unwrap();

            let path_width = map.vert_road_width.max(map.horiz_road_width) / 2;
            let on_outer = calc_occupied_perim_map(&agents, &map.vehicle_outer_pts, path_width, false);
            let on_inner = calc_occupied_perim_map(&agents, &map.vehicle_inner_pts, path_width, true);
            let pose = spawn_on_two_perims(&map, &map.buildings, BUILDING_VEHICLE_MARGIN,
                                           (VEHICLE_WIDTH, VEHICLE_LENGTH),
                                           &map.vehicle_outer_pts, &map.vehicle_inner_pts,
                                           &on_outer, &on_inner, SPAWN_MARGIN);

            // println!("Inner points: {:?}", map.vehicle_inner_pts);

            if let Some(pose) = pose {
                // can only be on one of the two lanes/paths
                for agent in agents.vehicles.iter() {
                    assert_ne!(Pose{x: agent.pose.0, y: agent.pose.1, theta: agent.pose.2},
                               Pose{x: pose.0, y: pose.1, theta: pose.2});
                }

                let mut agent = &mut agents.vehicles[agent_i];
                println!("Drew pose: {:?} and vehicle: {:?}", pose, agent);
                agent.on_map = true;
                agent.pose = pose;
            }
        }
    }
}

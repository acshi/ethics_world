use super::*;
use std::collections::HashSet;
use std::hash::{Hash, Hasher};

#[derive(PartialEq, Clone, Debug)]
struct Pose {
    x: u32,
    y: u32,
    theta: u32,
}

impl Eq for Pose {}

impl Hash for Pose {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
        self.theta.hash(state);
    }
}

#[test]
fn test_precomputed_sin_cos() {
    let range = TWO_PI_INCS as i32;
    for t in -range..range {
        let theta = t as f32 * THETA_PER_INC;
        let sin_cos1 = theta.sin_cos();
        let sin_cos2 = fast_sin_cos(t);
        assert!((sin_cos1.0 - sin_cos2.0).abs() < 1e-5);
        assert!((sin_cos1.1 - sin_cos2.1).abs() < 1e-5);
    }
}

#[test]
fn test_precompute_agent_dists() {
    let mut map = create_map();
    setup_map_paths(&mut map);
    let agents = create_agents(&mut map);

    let agent_dists = compute_agent_1_dists(&agents);
    for i in 0..agents.len() {
        let a1 = &agents[i];
        if !a1.on_map {
            continue;
        }
        for j in (i + 1)..agents.len() {
            let a2 = &agents[j];
            if !a2.on_map {
                continue;
            }
            let from_table = get_agent_1_dist(&agent_dists, i, j);
            let direct = obj_dist_1(a1.pose, (a1.width, a1.length),
                                    a2.pose, (a2.width, a2.length));
            println!("At {}, {} we have table {} and direct {}", i, j, from_table, direct);
            assert_eq!(from_table, direct);
        }
    }
}

#[test]
fn test_simple_collision_detection() {
    let agents = [Agent{pose: (10, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (14, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 0);

    let agents = [Agent{pose: (10, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (13, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);

    let agents = [Agent{pose: (10, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (6, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 0);

    let agents = [Agent{pose: (10, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (7, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);

    let agents = [Agent{pose: (10, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (10, 20, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 0);

    let agents = [Agent{pose: (10, 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (10, 19, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);
}

#[test]
fn test_rotation_collision_detection() {
    let agents = [Agent{pose: (43, 37, 1), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (48, 48, 1), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 0);

    let agents = [Agent{pose: (43, 37, 1), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()},
                  Agent{pose: (47, 46, TWO_PI_INCS - 2), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()}];
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);
}

#[test]
fn test_wall_collision_detection() {
    let mut agents = create_edge_walls();
    agents.push(Agent{pose: (20, MAP_HEIGHT - 10, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()});
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);

    let mut agents = create_edge_walls();
    agents.push(Agent{pose: (20, MAP_HEIGHT - 11, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()});
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 0);

    let mut agents = create_edge_walls();
    agents.push(Agent{pose: (1, 0, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()});
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);

    let mut agents = create_edge_walls();
    agents.push(Agent{pose: (0, 1, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()});
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);

    let mut agents = create_edge_walls();
    agents.push(Agent{pose: (1, 1, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()});
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 0);

    let mut agents = create_edge_walls();
    agents.push(Agent{pose: (MAP_WIDTH - 5, 1, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()});
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 0);

    let mut agents = create_edge_walls();
    agents.push(Agent{pose: (MAP_WIDTH - 4, 1, 0), width: 4, length: 10, kind: AgentKind::Vehicle, on_map: true, ..Agent::default()});
    let collisions = find_collisions(&agents);
    assert_eq!(collisions.len(), 1);
}

#[test]
fn test_initially_unoccupied() {
    let mut map = create_map();
    setup_map_paths(&mut map);

    let agents = create_agents(&mut map);

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

        let mut agents = create_agents(&mut map);
        setup_agents(&map, &mut agents);

        let mut all_poses = agents.iter().filter(|a| a.on_map && a.kind != AgentKind::Obstacle)
                                  .map(|a| Pose{x: a.pose.0, y: a.pose.1, theta: a.pose.2}).collect::<Vec<_>>();
        let all_poses_len = all_poses.len();
        let set_poses = all_poses.drain(..).collect::<HashSet<_>>(); // remove duplicate values

        assert_eq!(all_poses_len, set_poses.len());
    }
}

#[test]
fn test_spawn_pedestrian() {
    for i in 0..100 {
        println!("\nAttempt #: {}", i);

        let mut map = create_map();
        setup_map_paths(&mut map);
        let mut agents = create_agents(&mut map);

        let agent_i = draw_off_map_agent_i(&mut agents, AgentKind::Pedestrian);
        if agent_i.is_none() {
            continue; // no agents left to draw
        }
        let agent_i = agent_i.unwrap();

        let path_width = map.vert_sidewalk_width.max(map.horiz_sidewalk_width);
        let on_outer = calc_occupied_perim_map(&agents, &map.pedestrian_outer_pts, path_width, false);
        let on_inner = calc_occupied_perim_map(&agents, &map.pedestrian_inner_pts, path_width, true);

        let spawn_result = spawn_on_two_perims(&map, &map.buildings, BUILDING_PEDESTRIAN_MARGIN,
                                           (PEDESTRIAN_SIZE, PEDESTRIAN_SIZE),
                                           &map.pedestrian_outer_pts, &map.pedestrian_inner_pts,
                                           &on_outer, &on_inner, SPAWN_MARGIN);

        println!("Inner points: {:?}", map.pedestrian_inner_pts);
        println!("Outer points: {:?}", map.pedestrian_outer_pts);

        if let Some((pose, building_i)) = spawn_result {
            let mut agent = &mut agents[agent_i];
            println!("Drew pose: {:?} and pedestrian: {:?}", pose, agent);
            agent.on_map = true;
            agent.pose = pose;
            agent.source_building_i = building_i;

            let (width, length) = (agent.width as f32, agent.length as f32);
            let (x, y, theta) = (agent.pose.0 as f32, agent.pose.1 as f32, agent.pose.2);
            let theta = theta as i32;
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

        let occ_count_outer = on_outer.iter().filter(|&&b| b).count() as u32;
        let occ_count_inner = on_inner.iter().filter(|&&b| b).count() as u32;

        // can only be on one of the two lanes/paths
        assert!(occ_count_inner == 0 || occ_count_outer == 0);
        let total_count = occ_count_inner + occ_count_outer;
        // will be one extra when the pedestrian is right on the corner of the perimeter
        assert!(total_count == PEDESTRIAN_SIZE || total_count == PEDESTRIAN_SIZE + 1);
    }
}

#[test]
fn test_spawn_vehicle() {
    for i in 0..10 {
        println!("\nAttempt #: {}", i);

        let mut map = create_map();
        setup_map_paths(&mut map);
        let mut agents = create_agents(&mut map);

        // replenish_pedestrians(&map, &mut agents);

        for _ in 0..VEHICLE_N {
            let agent_i = draw_off_map_agent_i(&mut agents, AgentKind::Pedestrian);
            if agent_i.is_none() {
                continue; // no agents left to draw
            }
            let agent_i = agent_i.unwrap();

            let path_width = map.vert_road_width.max(map.horiz_road_width) / 2;
            let on_outer = calc_occupied_perim_map(&agents, &map.vehicle_outer_pts, path_width, false);
            let on_inner = calc_occupied_perim_map(&agents, &map.vehicle_inner_pts, path_width, true);
            let spawn_result = spawn_on_two_perims(&map, &map.buildings, BUILDING_VEHICLE_MARGIN,
                                               (VEHICLE_WIDTH, VEHICLE_LENGTH),
                                               &map.vehicle_outer_pts, &map.vehicle_inner_pts,
                                               &on_outer, &on_inner, SPAWN_MARGIN);

            // println!("Inner points: {:?}", map.vehicle_inner_pts);

            if let Some((pose, building_i)) = spawn_result {
                // can only be on one of the two lanes/paths
                for agent in agents.iter() {
                    assert_ne!(Pose{x: agent.pose.0, y: agent.pose.1, theta: agent.pose.2},
                               Pose{x: pose.0, y: pose.1, theta: pose.2});
                }

                let mut agent = &mut agents[agent_i];
                println!("Drew pose: {:?} and vehicle: {:?}", pose, agent);
                agent.on_map = true;
                agent.pose = pose;
                agent.source_building_i = building_i;
            }
        }
    }
}

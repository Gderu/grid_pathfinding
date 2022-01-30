use rand::Rng;
use rand::prelude::ThreadRng;
use std::ops::Range;
use std::f64::consts::PI;
use std::collections::HashMap;

const NUM_SHAPES: usize = 50;
const MAP_WIDTH: f64 = 1000.;
const MAP_HEIGHT: f64 = 1000.;
const RADIUS_RANGE: Range<i32> = 100..200;

#[derive(Debug)]
struct Line {
    origin: (f64, f64),
    ray: (f64, f64),
    target: (f64, f64),
}

struct Map {
    map: Vec<Line>,
    agent_pos: (f64, f64),
    agent_walls: Option<(usize, usize)>,
    goal_pos: (f64, f64),
    original_agent_pos: (f64, f64),
}

impl Map {
    fn init() -> Map {
        let mut rng = rand::thread_rng();
        let target1 = (100.0, 100.0);
        let target2 = (-100.0, 0.0);
        // let map = vec![vec![Line {
        //     origin: (0.0, 0.0),
        //     ray: target1,
        //     target: target1
        // }, Line {
        //     origin: target2,
        //     ray: (-target2.0, -target2.1),
        //     target: (0.0, 0.0)
        // }, Line {
        //     origin: target1,
        //     ray: (target2.0 - target1.0, target2.1 - target1.1),
        //     target: target2
        // }]];
        let map = (0..NUM_SHAPES).map(|_i| Map::create_shape(&mut rng)).collect::<Vec<Vec<Line>>>();
        // let agent_pos = (0.0, 0.0);
        let agent_pos = Map::pick_point(&map, &mut rng);
        // let goal_pos = (-100.0, -1.0);
        let goal_pos = Map::pick_point(&map, &mut rng);
        Map {
            map: map.into_iter().flat_map(|l| l).collect(),
            agent_pos,
            agent_walls: None,
            original_agent_pos: agent_pos,
            // agent_walls: Some((0, 1)),
            goal_pos,
        }
    }

    fn pick_point(map: &Vec<Vec<Line>>, rng: &mut ThreadRng) -> (f64, f64) {
        let mut point;
        let ray = (0., 1.0);
        let mut reroll = false;
        loop {
            point = ((rng.gen::<f64>() - 0.5) * MAP_WIDTH, (rng.gen::<f64>() - 0.5) * MAP_HEIGHT);
            for shape in map {
                let mut count = 0;
                for line in shape {
                    if (line.ray.0 * ray.1 - line.ray.1 * ray.0).abs() < 0.001 {
                        continue;
                    }
                    let t2 = (ray.0 * (line.origin.1 - point.1) + ray.1 * (point.0 - line.origin.0)) / (line.ray.0 * ray.1 - line.ray.1 * ray.0);
                    let t1 = if ray.0.abs() >= 0.0001 {
                        (line.origin.0 + line.ray.0 * t2 - point.0) / ray.0
                    } else if ray.1.abs() >= 0.0001 {
                        (line.origin.1 + line.ray.1 * t2 - point.1) / ray.1
                    } else {
                        continue;
                    };
                    if t1 >= 0. && t2 >= 0. && 1. >= t2 {//if the ray intersects with the line segment
                        count += 1;
                    }
                }
                if count % 2 == 1 { //the point is inside of the shape
                    reroll = true;
                    break;
                }
            }
            if reroll {
                reroll = false;
                continue;
            }
            return point;
        }
    }

    fn create_shape(rng: &mut ThreadRng) -> Vec<Line> {
        let num_vertices = rng.gen_range(3..7);
        let len_radius = rng.gen_range(RADIUS_RANGE) as f64;
        let theta = 2. * std::f64::consts::PI / num_vertices as f64;
        let degree_offset = rng.gen::<f64>() * theta;
        let center_pos = ((rng.gen::<f64>() - 0.5) * MAP_WIDTH, (rng.gen::<f64>() - 0.5) * MAP_HEIGHT);
        let mut result = Vec::with_capacity(num_vertices);
        let mut prev_point = (center_pos.0 + (len_radius * f64::cos(degree_offset + (num_vertices - 1) as f64 * theta)), center_pos.1 + (len_radius * f64::sin(degree_offset + (num_vertices - 1) as f64 * theta)));
        let mut new_point;
        for angle in (0..num_vertices).map(|i| degree_offset + i as f64 * theta) {
            new_point = (center_pos.0 + (len_radius * f64::cos(angle as f64)), center_pos.1 + (len_radius * f64::sin(angle as f64)));
            result.push(Line {
                origin: prev_point,
                ray: (new_point.0 - prev_point.0, new_point.1 - prev_point.1),
                target: new_point,
            });
            prev_point = new_point;
        }
        result
    }

    //gets points that the agent can see from its current position
    fn get_agent_view(&self) ->  Vec<(f64, f64)> {
        let mut result = vec![];
        self.find_closest_point(self.goal_pos, &mut result);
        for point in self.map.iter().map(|line| line.origin) {
            self.find_closest_point(point, &mut result);
        }
        result
    }

    //finds the closest point on a shape in the direction of variable point. Adds said point to result. Calls itself if the point found was a vertices with another point behind it.
    //If the target point is on the opposite side of the wall the agent is currently on, will not add point to result
    fn find_closest_point(&self, point: (f64, f64), result: &mut Vec<(f64, f64)>) {//(80, -200)
        let mut shortest_dist = None;
        let mut closest_point_ray = None;
        let ray = (point.0 - self.agent_pos.0, point.1 - self.agent_pos.1);
        let mut possibly_add_later = vec![];
        let mut in_goal = false;

        if dist(point, self.agent_pos) < 0.0001 {
            return;
        }

        if !self.is_legal_ray(ray) {
            // println!("Illegal ray: {:?}", point);
            return;
        }

        if dist(point, self.goal_pos) < 0.0001 {//if the point is the goal, make the default value be the distance to it and it being the target. If there is a line between the goal and the current position, it will override this
            // println!("In goal check");
            in_goal = true;
            shortest_dist = Some(1.);
            closest_point_ray = Some((point, 0., &self.map[0]));
        }

        for line in &self.map {
            if (line.ray.0 * ray.1 - line.ray.1 * ray.0).abs() < 0.001 {//if the ray and the line are parallel
                // if (line.ray.0 > 0.0) != (ray.0 > 0.0) {//if both rays are in opposite directions
                //     println!("{:?}, {:?}, {:?}", line, ray, point);
                //     continue;
                // } else {
                //     possibly_add_later.push(line);
                // }
                possibly_add_later.push(line);
            } else {
                let t2 = (ray.0 * (line.origin.1 - self.agent_pos.1) + ray.1 * (self.agent_pos.0 - line.origin.0)) / (line.ray.0 * ray.1 - line.ray.1 * ray.0);
                let t1 = if ray.0.abs() >= 0.0001 {
                    (line.origin.0 + line.ray.0 * t2 - self.agent_pos.0) / ray.0
                } else if ray.1.abs() >= 0.0001 {
                    (line.origin.1 + line.ray.1 * t2 - self.agent_pos.1) / ray.1
                } else {
                    continue;
                };
                if t1 > 0.0001 && t2 >= -0.00001 && 1.00001 >= t2 && shortest_dist.filter(|&val| t1 > val).is_none() {//if the point is on the line in the shape and is the closest point found so far
                    let t2 = if t2.abs() < 0.0001 {
                        0.
                    } else if (t2 - 1.).abs() < 0.0001 {
                        1.
                    } else {
                        t2
                    };
                    shortest_dist = Some(t1);
                    closest_point_ray = Some(((self.agent_pos.0 + t1 * ray.0, self.agent_pos.1 + t1 * ray.1), t2, line));
                    if in_goal {
                        // println!("{:?}", line);
                    }
                }
            }
        }
        if let Some((closest_point, t2, line)) = closest_point_ray {
            for line_to_add in possibly_add_later {
                if dist(self.agent_pos, closest_point) > dist(self.agent_pos, line_to_add.target) && dist( self.agent_pos, line_to_add.target) >= 0.0001 &&
                    result.iter().find(|&&p| dist(line_to_add.target, p) < 0.0001).is_none() {
                    result.push(line_to_add.target);
                }
                if dist(self.agent_pos, closest_point) > dist(self.agent_pos, line_to_add.origin) && dist( self.agent_pos, line_to_add.origin) >= 0.0001 &&
                    result.iter().find(|&&p| dist(line_to_add.origin, p) < 0.0001).is_none() {
                    result.push(line_to_add.origin);
                }
            }
            if !(t2 <= 1.0001 && t2 >= 0.9999 || t2 <= 0.00001 && t2 >= -0.00001) {
                return;
            }
            if result.iter().find(|&&a| dist(closest_point, a) < 0.0001).is_some() {
                // println!("Already contains: {:?}, {:?}", line, point);
            }
            result.push(closest_point);
        } else {
            for line_to_add in possibly_add_later {
                if dist( self.agent_pos, line_to_add.target) >= 0.0001 && result.iter().find(|&&p| dist(line_to_add.target, p) < 0.0001).is_none() {
                    result.push(line_to_add.target);
                }
                if dist( self.agent_pos, line_to_add.origin) >= 0.0001 && result.iter().find(|&&p| dist(line_to_add.origin, p) < 0.0001).is_none() {
                    result.push(line_to_add.origin);
                }
            }
        }
    }

    //is the agent on the side of the wall that allows this ray or not
    fn is_legal_ray(&self, to_check: (f64, f64)) -> bool {
        if let Some(agent_walls) = self.agent_walls {
            let ray1 = self.map[agent_walls.0].ray;
            let ray2 = (-self.map[agent_walls.1].ray.0, -self.map[agent_walls.1].ray.1);
            let thetas = [f64::atan2(ray1.1, ray1.0), f64::atan2(ray2.1, ray2.0)];
            let (theta_max, theta_min) = if thetas[0] > thetas[1] {
                (thetas[0], thetas[1])
            } else {
                (thetas[1], thetas[0])
            };
            let &theta_diff = [theta_max - theta_min, theta_min + 2. * PI - theta_max].iter().min_by(|a, b| a.partial_cmp(&b).unwrap()).unwrap();
            let ray_angle = f64::atan2(to_check.1, to_check.0);
            if (theta_min - ray_angle).abs() < 0.001 || (theta_max - ray_angle).abs() < 0.001 {
                true
            } else if theta_min < ray_angle && ray_angle < theta_max {
                theta_diff == theta_min + 2. * PI - theta_max
            } else {
                theta_diff == theta_max - theta_min
            }
        } else {
            true //the agent is not on a wall, so every move should be legal
        }
    }

    fn get_agent_pos(&self) -> (f64, f64) {
        self.agent_pos
    }

    fn get_goal_pos(&self) -> (f64, f64) {
        self.goal_pos
    }

    /*Moves the agent to a point slightly before target.
    Target has to have been a position returned from get_agent_view - the function does not check its validity.
    Returns the distance to target.*/
    fn move_agent(&mut self, target: (f64, f64)) -> f64 {
        let distance = ((target.1 - self.agent_pos.1).powf(2.) + (target.0 - self.agent_pos.0).powf(2.)).sqrt();
        let mut counter = 0;
        let mut new_agent_walls = (0, 0);
        for (i, line) in self.map.iter().enumerate() {
            if dist(line.origin, target) < 0.001 {
                new_agent_walls.0 = i;
                counter += 1;
            } else if dist(line.target, target) < 0.001 {
                new_agent_walls.1 = i;
                counter += 1;
            }
            if counter == 2 {
                self.agent_walls = Some(new_agent_walls);
                break;
            }
        }
        self.agent_pos = target;
        return distance
    }

    //reset the map to create a new one with new data and positions
    fn reset(&mut self) {
        *self = Map::init();
    }

    fn reached_goal(&self) -> bool {
        dist(self.agent_pos, self.goal_pos) < 0.01
    }

    fn reset_agent_pos(&mut self) {
        self.agent_pos = self.original_agent_pos;
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powf(2.) + (a.1 - b.1).powf(2.)).sqrt()
}

fn hill_climbing_search(children: Vec<(f64, f64)>, curr_pos: (f64, f64), goal_pos: (f64, f64)) -> (f64, f64) {
    let h_n = |point| dist(curr_pos, point) + dist(point, goal_pos) * 2.;
    *children.iter()
        .filter(|&&a| dist(a, curr_pos) > 0.0001)
        .min_by(|&&a, &&b| h_n(a).partial_cmp(&h_n(b)).expect("No NaNs")).expect("Should never happens")
}

fn ltra(children: Vec<(f64, f64)>, curr_pos: (f64, f64), goal_pos: (f64, f64), cost_table: &mut HashMap<(i32, i32), f64>) -> (f64, f64) {
    let h_n = |point| dist(curr_pos, point) + dist(point, goal_pos) * 2.;
    let mut choice = None;
    let mut lowest_cost = None;
    for child in children {
        let rounded_child = ((child.0 * 1000.).round() as i32, (child.1 * 1000.).round() as i32);
        if !cost_table.contains_key(&rounded_child) {
            cost_table.insert(rounded_child, h_n(child));
        }
        if lowest_cost.filter(|&v| cost_table[&rounded_child] as f64 > v).is_none() {//if lowest_cost is undefined or higher than child's cost
            lowest_cost = Some(cost_table[&rounded_child]);
            choice = Some(child);
        }
    }
    let (choice, lowest_cost) = choice.zip(lowest_cost).expect(format!("No children to {:?}", curr_pos).as_str());
    let rounded_curr_pos = ((curr_pos.0 * 1000.).round() as i32, (curr_pos.1 * 1000.).round() as i32);
    cost_table.insert(rounded_curr_pos, lowest_cost + dist(choice, curr_pos));
    choice
}

fn main() {
    let mut map = Map::init();
    let mut score_hill_climbing = 0.;
    let mut score_ltra = 0.;
    let mut num_solutions_hill_climbing = 0;
    let mut num_solutions_ltra = 0;
    for line in &map.map {
        println!("\\operatorname{{polygon}}({:?},{:?})", line.origin, line.target);
    }
    println!("{:?}", map.get_agent_pos());
    println!("{:?}", map.get_goal_pos());
    for _ in 0..1 {
        let (is_valid_hill_climbing, new_score_hill_climbing) = hill_climbing_algorithm(&mut map);
        map.reset_agent_pos();
        let (is_valid_ltra, new_score_ltra) = ltra_algorithm(&mut map);
        if let Some(path) = is_valid_ltra {
            println!("LTRA* solution:");
            for point in path {
                println!("{:?}", point);
            }
            score_ltra += new_score_ltra;
            num_solutions_ltra += 1;
        }
        if let Some(path) = is_valid_hill_climbing {
            println!("Hill Climbing solution:");
            for point in path {
                println!("{:?}", point);
            }
            score_hill_climbing += new_score_hill_climbing;
            num_solutions_hill_climbing += 1;
        } else {
            println!("Hill climbing couldn't find solution");
        }
    }
    println!("Hill Climbing score: {:.5}\nHill Climbing solutions: {}\n\nLTRA* score: {:.5}\nLTRA* solutions: {}", score_hill_climbing, num_solutions_hill_climbing, score_ltra, num_solutions_ltra);
}

fn ltra_algorithm(map: &mut Map) -> (Option<Vec<(f64, f64)>>, f64) {
    let mut cost_table = HashMap::new();
    let mut path = vec![map.get_agent_pos()];
    let mut score = 0.;
    loop {
        let target = ltra(map.get_agent_view(), map.get_agent_pos(), map.get_goal_pos(), &mut cost_table);
        score -= map.move_agent(target);
        path.push(map.get_agent_pos());
        if map.reached_goal() {
            score += 1000.;
            return (Some(path), score);
        }
    }
}

fn hill_climbing_algorithm(map: &mut Map) -> (Option<Vec<(f64, f64)>>, f64) {
    let mut score = 0.;
    let mut path = vec![map.get_agent_pos()];
    let mut remember = vec![];
    loop {
        let target = hill_climbing_search(map.get_agent_view(), map.get_agent_pos(), map.get_goal_pos());
        if remember.iter().find(|&&p| dist(p, target) < 0.001).is_some() {

            return (None, score);
        } else {
            remember.push(target);
        }
        score -= map.move_agent(target);

        path.push(map.get_agent_pos());
        if map.reached_goal() {
            score += 1000.;
            return (Some(path), score);
        }
    }
}
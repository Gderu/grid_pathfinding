use rand::Rng;
use rand::prelude::ThreadRng;
use std::ops::Range;
use std::io;
use std::f32::consts::PI;

const NUM_SHAPES: usize = 5;
const MAP_WIDTH: f32 = 1000.;
const MAP_HEIGHT: f32 = 1000.;
const RADIUS_RANGE: Range<i32> = 100..200;

#[derive(Debug)]
struct Line {
    origin: (f32, f32),
    ray: (f32, f32),
    target: (f32, f32),
}

struct Map {
    map: Vec<Line>,
    agent_pos: (f32, f32),
    agent_walls: Option<(usize, usize)>,
    goal_pos: (f32, f32),
}

impl Map {
    fn init() -> Map {
        let mut rng = rand::thread_rng();
        let map = (0..NUM_SHAPES).map(|_i| Map::create_shape(&mut rng)).collect::<Vec<Vec<Line>>>();
        let agent_pos = Map::pick_point(&map, &mut rng);
        let goal_pos = Map::pick_point(&map, &mut rng);
        Map {
            map: map.into_iter().flat_map(|l| l).collect(),
            agent_pos,
            agent_walls: None,
            goal_pos,
        }
    }

    fn pick_point(map: &Vec<Vec<Line>>, rng: &mut ThreadRng) -> (f32, f32) {
        let mut point;
        let ray = (0., 1.0);
        let mut reroll = false;
        loop {
            point = ((rng.gen::<f32>() - 0.5) * MAP_WIDTH, (rng.gen::<f32>() - 0.5) * MAP_HEIGHT);
            for shape in map {
                let mut count = 0;
                for line in shape {
                    if line.ray.0 * ray.1 - line.ray.1 * ray.0 == 0. {
                        continue;
                    }
                    let t2 = (ray.0 * (line.origin.1 - point.1) + ray.1 * (point.0 - line.origin.0)) / (line.ray.0 * ray.1 - line.ray.1 * ray.0);
                    let t1 = if ray.0 != 0. {
                        (line.origin.0 + line.ray.0 * t2 - point.0) / ray.0
                    } else if ray.1 != 0. {
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
        let len_radius = rng.gen_range(RADIUS_RANGE) as f32;
        let theta = 2. * std::f32::consts::PI / num_vertices as f32;
        let degree_offset = rng.gen::<f32>() * theta;
        let center_pos = ((rng.gen::<f32>() - 0.5) * MAP_WIDTH, (rng.gen::<f32>() - 0.5) * MAP_HEIGHT);
        let mut result = Vec::with_capacity(num_vertices);
        let mut prev_point = (center_pos.0 + (len_radius * f32::cos(degree_offset + (num_vertices - 1) as f32 * theta)), center_pos.1 + (len_radius * f32::sin(degree_offset + (num_vertices - 1) as f32 * theta)));
        let mut new_point;
        for angle in (0..num_vertices).map(|i| degree_offset + i as f32 * theta) {
            new_point = (center_pos.0 + (len_radius * f32::cos(angle as f32)), center_pos.1 + (len_radius * f32::sin(angle as f32)));
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
    fn get_agent_view(&self) ->  Vec<(f32, f32)> {
        let mut result = vec![];
        self.find_closest_point(self.goal_pos, &mut result, false);
        for point in self.map.iter().map(|line| line.origin) {
            self.find_closest_point(point, &mut result, true);
        }
        result
    }

    //finds the closest point on a shape in the direction of variable point. Adds said point to result. Calls itself if the point found was a vertices with another point behind it.
    //If the target point is on the opposite side of the wall the agent is currently on, will not add point to result
    fn find_closest_point(&self, point: (f32, f32), result: &mut Vec<(f32, f32)>, recursive: bool) {
        let mut shortest_dist = -1.;
        let mut closest_point_ray = None;
        let ray = (point.0 - self.agent_pos.0, point.1 - self.agent_pos.1);

        if !self.is_legal_ray(ray) {
            return;
        }

        if point == self.goal_pos {//if the point is the goal, make the default value be the distance to it and it being the target. If there is a line between the goal and the current position, it will override this
            shortest_dist = 1.;
            closest_point_ray = Some((point, (0., 0.), (0., 0.), 0.));
        }

        for line in &self.map {
            if line.ray.0 * ray.1 - line.ray.1 * ray.0 == 0. {
                continue;
            }
            let t2 = (ray.0 * (line.origin.1 - self.agent_pos.1) + ray.1 * (self.agent_pos.0 - line.origin.0)) / (line.ray.0 * ray.1 - line.ray.1 * ray.0);
            let t1 = if ray.0 != 0. {
                (line.origin.0 + line.ray.0 * t2 - self.agent_pos.0) / ray.0
            } else if ray.1 != 0. {
                (line.origin.1 + line.ray.1 * t2 - self.agent_pos.1) / ray.1
            } else {
                continue;
            };
            if t1 > 0. && t2 >= 0. && 1. >= t2 &&
                (shortest_dist == -1. || shortest_dist > t1) {//if the point is on the line in the shape and is the closest point found so far

                shortest_dist = t1;
                closest_point_ray = Some(((self.agent_pos.0 + t1 * ray.0, self.agent_pos.1 + t1 * ray.1),
                    line.origin,
                    line.ray,
                    t2,
                ));
            }
        }
        if let Some((closest_point, origin, ray, t2)) = closest_point_ray {
            if t2 != 1. && t2 != 0. {
                return;
            }
            result.push(closest_point);
            if recursive {
                return;
            }
            if closest_point == origin {
                self.find_closest_point((origin.0 - ray.0 * 0.0001, origin.1 - ray.1 * 0.0001), result, false);
            } else if closest_point == (origin.0 + ray.0, origin.1 + ray.1) {
                self.find_closest_point((origin.0 + ray.0 * 1.0001, origin.1 + ray.1 * 1.0001), result, false);
            }
        }
    }

    fn is_legal_ray(&self, to_check: (f32, f32)) -> bool {
        if let Some(agent_walls) = self.agent_walls {
            let ray1 = self.map[agent_walls.0].ray;
            let ray2 = (-self.map[agent_walls.1].ray.0, -self.map[agent_walls.1].ray.1);
            let thetas = [f32::acos(ray1.1 / dist((0., 0.), ray1)), f32::acos(ray2.1 / dist((0., 0.), ray2))];
            let (theta_max, theta_min) = if thetas[0] > thetas[1] {
                (thetas[0], thetas[1])
            } else {
                (thetas[1], thetas[0])
            };
            let &theta_diff = [theta_max - theta_min, theta_max + theta_min - 2. * PI].iter().min_by(|a, b| a.partial_cmp(&b).unwrap()).unwrap();
            let ray_angle = f32::acos(to_check.1 / dist((0., 0.), to_check));

            (theta_min < ray_angle && ray_angle < theta_max) == (theta_diff == theta_max - theta_min)//true if ray_angle is between the smaller angle of theta_max and theta_min
        } else {
            true //the agent is not on a wall, so every move should be legal
        }
    }

    fn get_agent_pos(&self) -> (f32, f32) {
        self.agent_pos
    }

    fn get_goal_pos(&self) -> (f32, f32) {
        self.goal_pos
    }

    /*Moves the agent to a point slightly before target.
    Target has to have been a position returned from get_agent_view - the function does not check its validity.
    Returns the distance to target.*/
    fn move_agent(&mut self, target: (f32, f32)) -> f32 {
        let distance = ((target.1 - self.agent_pos.1).powf(2.) + (target.0 - self.agent_pos.0).powf(2.)).sqrt();
        let mut counter = 0;
        let mut new_agent_walls = (0, 0);
        for (i, line) in self.map.iter().enumerate() {
            if line.origin == target {
                new_agent_walls.0 = i;
                counter += 1;
            } else if line.target == target {
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
}

fn dist(a: (f32, f32), b: (f32, f32)) -> f32 {
    ((a.0 - b.0).powf(2.) + (a.1 - b.1).powf(2.)).sqrt()
}

fn hill_climbing_search(children: Vec<(f32, f32)>, curr_pos: (f32, f32), goal_pos: (f32, f32)) -> (f32, f32) {
    let h_n = |point| dist(curr_pos, point) + dist(point, goal_pos) * 2.;
    println!("{:?}", &children);
    *children.iter()
        .filter(|&&a| dist(a, curr_pos) > 0.0001)
        .min_by(|&&a, &&b| h_n(a).partial_cmp(&h_n(b)).expect("No NaNs")).expect("Should never happens")

}

fn main() {
    let mut map = Map::init();
    println!("Shapes:");
    for l in &map.map {
        println!("\\operatorname{{polygon}}({:?}, {:?})", l.origin, l.target);
    }
    println!("Agent pos: {:?}", map.get_agent_pos());
    println!("Goal pos: {:?}", map.get_goal_pos());
    let mut target;
    let mut score = 0.;
    let mut i = 0;
    let mut path = vec![map.get_agent_pos()];
    let mut counter = 0;
    loop {
        target = hill_climbing_search(map.get_agent_view(), map.get_agent_pos(), map.get_goal_pos());
        if target == map.get_goal_pos() && counter == 0 {
            main();
            return;
        }
        score -= map.move_agent(target);
        //println!("target: {:?}", map.get_agent_pos());

        path.push(map.get_agent_pos());
        //io::stdin().read_line(&mut String::new());
        if map.reached_goal() {
            println!("IN");
            println!("Path:");
            for point in path {
                println!("{:?}", point);
            }
            score += 1000.;
            map.reset();
            i += 1;
            path = vec![map.get_agent_pos()];
        }
        if i == 1 {
            break;
        }
        counter += 1;
    }
}
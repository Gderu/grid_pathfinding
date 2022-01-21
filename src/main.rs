use rand::Rng;
use rand::prelude::ThreadRng;
use array_init;
use std::ops::Range;

const NUM_SHAPES: usize = 1;
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
    map: [Vec<Line>; NUM_SHAPES],
    agent_pos: (f32, f32),
    goal_pos: (f32, f32),
    rng: ThreadRng,
}

impl Map {
    fn init() -> Map {
        let mut rng = rand::thread_rng();
        Map {
            map: array_init::array_init(|_i: usize| Map::create_shape(&mut rng)),
            agent_pos: ((rng.gen::<f32>() - 0.5) * MAP_WIDTH, (rng.gen::<f32>() - 0.5) * MAP_HEIGHT),
            goal_pos: ((rng.gen::<f32>() - 0.5) * MAP_WIDTH, (rng.gen::<f32>() - 0.5) * MAP_HEIGHT),
            rng,
        }
    }

    fn create_shape(rng: &mut ThreadRng) -> Vec<Line> {
        let num_vertices = rng.gen_range(3..7);
        let len_radius = rng.gen_range(RADIUS_RANGE) as f32;
        let theta = 2. * std::f32::consts::PI / num_vertices as f32;
        let degree_offset = rng.gen::<f32>() * theta;
        let center_pos = ((rng.gen::<f32>() - 0.5) * MAP_WIDTH, (rng.gen::<f32>() - 0.5) * MAP_HEIGHT);
        let mut result = Vec::with_capacity(num_vertices);
        println!("Rad: {}, theta: {}, offset: {}, center: {:?}", len_radius, theta, degree_offset, center_pos);
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

    //gets the vertices that the agent can view from its current position
    fn get_agent_view(&self) ->  Vec<(f32, f32)> {
        unimplemented!();
    }

    fn get_agent_pos(&self) -> (f32, f32) {
        self.agent_pos
    }

    fn get_goal_pos(&self) -> (f32, f32) {
        self.goal_pos
    }

    /*Moves the agent to target.
    Target has to have been a position returned from get_agent_view - the function does not check its validity.
    Returns the distance to target.*/
    fn move_agent(&mut self, target: (f32, f32)) -> f32 {
        let distance = ((self.agent_pos.0 - target.0).powf(2.) + (self.agent_pos.1 - target.1).powf(2.)).sqrt();
        self.agent_pos = target;
        return distance
    }

    //reset the map to create a new one with new data and positions
    fn reset(&mut self) {
        *self = Map::init();
    }

    fn reached_goal(&self) -> bool {
        self.goal_pos == self.agent_pos
    }
}

fn hill_climbing_search(children: Vec<(f32, f32)>, curr_pos: (f32, f32), goal_pos: (f32, f32)) -> (f32, f32) {
    unimplemented!();
}

fn main() {
    let mut map = Map::init();
    let mut target;
    let mut score = 0.;
    let mut i = 0;
    let mut path = vec![map.get_agent_pos()];
    loop {
        target = hill_climbing_search(map.get_agent_view(), map.get_agent_pos(), map.get_goal_pos());
        score -= map.move_agent(target);
        path.push(target);
        if map.reached_goal() {
            for shape in map.map {
                println!("Shape:");
                for line in shape {
                    println!("{:?}", line.origin);
                }
            }
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
    }
}
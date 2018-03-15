use std::collections::BinaryHeap;
use std::collections::VecDeque;
use std::collections::HashSet;
use std::marker;
use std::cmp::Ordering;
use std::rc::Rc;
use std::hash::Hash;

pub struct PriorityQueue<T>(BinaryHeap<T>);
pub struct FifoQueue<T>(VecDeque<T>);
pub struct LifoQueue<T>(VecDeque<T>);

pub trait SearchQueue<T>
where T: Clone
{
    fn new() -> Self;
    fn add(&mut self, item: T);
    fn remove_next(&mut self) -> Option<T>;
}

impl<T> SearchQueue<T> for PriorityQueue<T>
where T: Clone + Ord
{
    fn new() -> Self { PriorityQueue::<T>(BinaryHeap::<T>::new()) }
    fn add(&mut self, item: T) { self.0.push(item) }
    fn remove_next(&mut self) -> Option<T> { self.0.pop() }
}

impl<T> SearchQueue<T> for FifoQueue<T>
where T: Clone
{
    fn new() -> Self { FifoQueue::<T>(VecDeque::<T>::new()) }
    fn add(&mut self, item: T) { self.0.push_back(item) }
    fn remove_next(&mut self) -> Option<T> { self.0.pop_front() }
}

impl<T> SearchQueue<T> for LifoQueue<T>
where T: Clone
{
    fn new() -> Self { LifoQueue::<T>(VecDeque::<T>::new()) }
    fn add(&mut self, item: T) { self.0.push_back(item) }
    fn remove_next(&mut self) -> Option<T> { self.0.pop_back() }
}

#[derive(Clone)]
pub struct SearchProblem<'a, T: 'a, QT>
    where T: Clone, QT: SearchQueue<SearchNode<T>>
{
	initial_state: T,
    expansion_count: i32,

    allow_cycles: bool,
    debugging: bool,

    use_iterative_depth: bool,
    iterative_depth_init: i32,
    iterative_depth_increment: i32,
    iterative_depth_limit: i32,
    current_depth_limit: i32,

    problem_traits: &'a SearchProblemTrait<T>,

    _marker: marker::PhantomData<QT>,
}

impl<'a, T: 'a, QT> SearchProblem<'a, T, QT>
    where T: Clone, QT: SearchQueue<SearchNode<T>>
{
    #[allow(dead_code)]
    pub fn get_expansion_count(&self) -> i32 {
        self.expansion_count
    }
}

#[allow(dead_code)]
pub fn create_search_problem<'a, T, QT>(initial_state: T, problem_traits: &'a SearchProblemTrait<T>,
                                    allow_cycles: bool, debugging: bool)
                                    -> SearchProblem<'a, T, QT>
    where T: Clone, QT: SearchQueue<SearchNode<T>>
{
    SearchProblem { initial_state, allow_cycles, debugging, problem_traits,
                    expansion_count: 0,
                    use_iterative_depth: false,
                    iterative_depth_init: 0,
                    iterative_depth_increment: 0,
                    iterative_depth_limit: 0,
                    current_depth_limit: 0,
                    _marker: marker::PhantomData::<QT>{},
    }
}

#[derive(Clone)]
pub struct SearchNode<T>
    where T: Clone
{
    pub parent: Option<Rc<SearchNode<T>>>,
    pub state: T,
    pub action: i32,
    pub depth: i32,
    pub path_cost: f32,
    pub ordering_cost: f32,
    pub expansion_count: i32,
}

impl<T> SearchNode<T>
    where T: Clone
{
    #[allow(dead_code)]
    pub fn get_expansion_count(&self) -> i32 {
        self.expansion_count
    }
}

impl<T> Ord for SearchNode<T>
    where T: Clone
{
    fn cmp(&self, other: &SearchNode<T>) -> Ordering {
        if self.ordering_cost == other.ordering_cost {
            Ordering::Equal
        } else {
            if self.ordering_cost < other.ordering_cost {
                Ordering::Less
            } else {
                Ordering::Greater
            }
        }
    }
}

impl<T> PartialOrd for SearchNode<T>
    where T: Clone
{
    fn partial_cmp(&self, other: &SearchNode<T>) -> Option<Ordering> {
        Some(self.cmp(&other))
    }
}

impl<T> PartialEq for SearchNode<T>
    where T: Clone
{
    fn eq(&self, other: &SearchNode<T>) -> bool {
        self.ordering_cost == other.ordering_cost
    }
}

impl<T> Eq for SearchNode<T> where T: Clone {}

pub trait SearchProblemTrait<T>
    where T: Clone
{
    fn is_goal(&self, node: &SearchNode<T>) -> bool;
    fn step_cost(&self, node: &SearchNode<T>, action: i32) -> f32;
    fn ordering_cost(&self, node: &SearchNode<T>) -> f32;
    fn expand_state(&self, node: &SearchNode<T>) -> Box<Iterator<Item = (T, i32)>>;
}

impl<'a, T, QT> SearchProblemTrait<T> for SearchProblem<'a, T, QT>
    where T: Clone, QT: SearchQueue<SearchNode<T>>
{
    fn is_goal(&self, node: &SearchNode<T>) -> bool {
        self.problem_traits.is_goal(node)
    }

    fn step_cost(&self, node: &SearchNode<T>, action: i32) -> f32 {
        self.problem_traits.step_cost(node, action)
    }

    fn ordering_cost(&self, node: &SearchNode<T>) -> f32 {
        self.problem_traits.ordering_cost(node)
    }

    fn expand_state(&self, node: &SearchNode<T>) -> Box<Iterator<Item = (T, i32)>> {
        self.problem_traits.expand_state(node)
    }
}

#[allow(dead_code)]
fn make_node<'a, T, QT>(p: &'a mut SearchProblem<T, QT>, parent: Option<Rc<SearchNode<T>>>, state: T, action: i32) -> SearchNode<T>
    where T: Clone, QT: SearchQueue<SearchNode<T>>
{
    let mut node = SearchNode {
        state, parent, action,
        depth: 0,
        path_cost: 0.0,
        ordering_cost: 0.0,
        expansion_count: p.expansion_count,
    };
    if let Some(ref parent) = node.parent {
        node.depth = parent.depth + 1;
        node.path_cost = parent.path_cost + p.step_cost(&node, action);
    }
    node.ordering_cost = p.ordering_cost(&node);
    node
}

// result error value indicates whether a depth increase could be helpful
#[allow(dead_code)]
fn inner_tree_search<'a, T, QT>(p: &'a mut SearchProblem<T, QT>) -> Result<SearchNode<T>, bool>
    where T: Clone + Hash + Eq, QT: SearchQueue<SearchNode<T>>
{
    let mut frontier = QT::new();
    let initial_state = p.initial_state.clone();
    frontier.add(make_node(p, None, initial_state, 0));

    let expanded_set = if p.allow_cycles { None } else { Some(HashSet::<T>::new()) };

    let mut needs_depth_increase = false;

    loop {
        let node = frontier.remove_next();
        if node.is_none() {
            return Err(needs_depth_increase);
        }
        let node = node.unwrap();

        if let Some(ref expanded_set) = expanded_set {
            if expanded_set.contains(&node.state) {
                continue;
            }
        }

        if p.debugging {
            println!("Expanding node with path_cost: {:.2} depth: {} ", node.path_cost, node.depth);
        }
        p.expansion_count += 1;

        if p.is_goal(&node) {
            if p.debugging {
                println!("");
            }
            return Ok(node);
        }

        if p.use_iterative_depth && node.depth >= p.current_depth_limit {
            if p.iterative_depth_increment > 0 {
                needs_depth_increase = true;
            }
            if p.debugging {
                println!("");
            }
            continue;
        }

        let mut expansion = p.expand_state(&node);
        let node = Rc::new(node);
        while let Some((new_state, action)) = expansion.next() {
            if let Some(ref expanded_set) = expanded_set {
                if expanded_set.contains(&new_state) {
                    continue;
                }
            }
            frontier.add(make_node(p, Some(node.clone()), new_state, action));
        }

        if p.debugging {
            println!("");
        }
    }
}

#[allow(dead_code)]
pub fn tree_search<'a, T, QT>(p: &'a mut SearchProblem<T, QT>) -> Option<SearchNode<T>>
    where T: Clone + Hash + Eq, QT: SearchQueue<SearchNode<T>>
{
    p.current_depth_limit = p.iterative_depth_init;
    let mut needs_depth_increase = false;
    let mut result = inner_tree_search(p);
    while result.is_err() && needs_depth_increase && p.current_depth_limit < p.iterative_depth_limit {
        p.current_depth_limit += p.iterative_depth_increment;
        println!("Increasing iterative-depth limit to {}", p.current_depth_limit);
        result = inner_tree_search(p);
        needs_depth_increase = if let Err(val) = result {val} else {false};
    }
    return result.ok()
}

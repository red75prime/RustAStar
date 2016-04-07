//! # AStar library
//!
//! AStar library provides implementation of A* algorithm.
//! Library interface is functional (no OOP).

extern crate typed_arena;

use typed_arena::Arena as TypedArena;
use std::collections::BinaryHeap;
use std::collections::BTreeSet;
use std::rc::Rc;
use std::cmp::Ordering;


struct FrontierElem<'a,A,S> where S:'a {
    prev : Option<Rc<FrontierElem<'a,A,S>>>,
    action : Option<A>,
    state : &'a S,
    cost : f32,
    total_cost : f32,
}

impl<'a,A,S> PartialEq for FrontierElem<'a,A,S> {
    fn eq(&self, other : &FrontierElem<'a,A,S>) -> bool {
        other.total_cost==self.total_cost
    }
}

impl<'a,A,S> PartialOrd for FrontierElem<'a,A,S> {
    // Ordering is reversed, for BinaryHeap to act as MinHeap.
    fn partial_cmp(&self, other : &FrontierElem<'a,A,S>) -> Option<Ordering> {
        other.total_cost.partial_cmp(&self.total_cost)
    }
}

impl<'a,A,S> Eq for FrontierElem<'a,A,S> {}

impl<'a,A,S> Ord for FrontierElem<'a,A,S> {
    fn cmp(&self, other : &FrontierElem<'a,A,S>) -> Ordering {
        other.total_cost.partial_cmp(&self.total_cost).unwrap() 
    }
}

/// Copies action fields from single-linked list to array in reverse order.
fn collect_actions<A,S>(fel: &Rc<FrontierElem<A,S>>) -> Vec<A>
    where A:Clone {
    let mut ret=Vec::<A>::with_capacity(16);
    let mut cur=fel;
    while let Some(ref prev)=cur.prev {
        if let Some(ref action)=cur.action {
            ret.push(action.clone());
        };
        cur=prev;
    }
    (&mut ret[..]).reverse();
    ret
}

/// Contains algorithm's options. Deprecated. Use `astar_opt_ex` function
///
#[derive(Debug,Clone,Copy)]
pub enum AstarOpt {
    /// Default options. Same as `AOMaxFrontierStateCap(std::usize::MAX, 128)`
    AONone, 
    /// Sets maximum size of frontier queue. Search functions produce `Err(FrontierLimit(_))` on overflow. Default value `std::usize::MAX`, effectively unbounded.
    AOMaxFrontier(usize),
    /// Sets initial size of states' arena. Default value 128. Suggested to be roughly equal to the expected count of explored states.
    AOStateCap(usize),
    /// Sets both maximum size of frontier queue and initial size of states' arena.
    AOMaxFrontierStateCap(usize,usize), 
}

use AstarOpt::{AONone, AOMaxFrontier, AOStateCap, AOMaxFrontierStateCap};

/// Search errors
///
#[derive(Debug,PartialEq,Eq)]
pub enum AstarError<A> {
    /// Path doesn't exist.
    NoPath,
    /// Size of frontier queue exceeded limit. Contains partial path, having minimal expected total cost.
    FrontierLimit(Vec<A>),
    /// Count of explored states exceeded limit. Contains partial path, having minimal expected total cost.
    StateLimit(Vec<A>),
    /// Heuristic cost function returned NaN.
    NanCost,
}

use AstarError::{NoPath, FrontierLimit, StateLimit, NanCost};

/// Searches for sequence of actions, which lead from state `s0` to goal state.
/// Goal state is a state `s`, such that `fend(s)` returns true.
/// `fnxt` closure should return vector of tuples (action, successor state, cost) for given state.
/// `fheu` closure should return estimated cost of reaching goal state for given state. Heuristic function must be admissible and constistent.
pub fn astar_opt_ex<S,A,Fnxt,Fend,Fheu>(s0 : S, fnxt : Fnxt, fend : Fend, fheu : Fheu, 
                                        arena_cap: usize, frontier_max_cnt: usize, state_max_cnt: usize
                                        ) -> Result<Vec<A>,AstarError<A>>
    where S : Ord, 
        A : Clone,
        Fnxt : Fn(&S) -> Vec<(A,S,f32)>,
        Fend : Fn(&S) -> bool,
        Fheu : Fn(&S) -> f32  {

    let state_arena=TypedArena::<S>::with_capacity(arena_cap);
    let mut frontier=BinaryHeap::<Rc<FrontierElem<A,S>>>::new();
    let mut visited=BTreeSet::<&S>::new();
    let mut stored_state_count: usize = 0;
    let rs0 = state_arena.alloc(s0);
    frontier.push(Rc::new(FrontierElem::<A,S>{prev:None, action:None, state:rs0, cost:0.0, total_cost:0.0}));
    while let Some(fnode)=frontier.pop() {
        let cstate=(*fnode).state;
        if fend(cstate) {
            return Ok(collect_actions(&fnode));
        }
        // never true if frontier_max_cnt equals std::usize::MAX
        if frontier.len()>frontier_max_cnt {
            return Err(FrontierLimit(collect_actions(&fnode)));
        }
        visited.insert(cstate);
        let nxts=fnxt(cstate);
        for (a,s,c) in nxts {
            if c.is_nan() {
                return Err(NanCost);
            }
            if !visited.contains(&s) {
                let rs=state_arena.alloc(s);
                stored_state_count += 1;
                if stored_state_count>state_max_cnt {
                    return Err(StateLimit(collect_actions(&fnode)));
                };
                let hcost=fheu(rs);
                frontier.push(Rc::new(FrontierElem{prev: Some(fnode.clone()), action: Some(a), state: rs, cost:(*fnode).cost+c, total_cost:(*fnode).cost+c+hcost}));
            }
        }
    }
    return Err(NoPath);
}
        
pub fn astar_opt<S,A,Fnxt,Fend,Fheu>(s0 : S, fnxt : Fnxt, fend : Fend, fheu : Fheu, opt: AstarOpt) -> Result<Vec<A>,AstarError<A>>
    where S : Ord, 
        A : Clone,
        Fnxt : Fn(&S) -> Vec<(A,S,f32)>,
        Fend : Fn(&S) -> bool,
        Fheu : Fn(&S) -> f32  {
    let (max_frontier, init_cap)=match opt {
        AONone =>                       (std::usize::MAX, 128),
        AOMaxFrontier(mf) =>            (mf, 128),
        AOStateCap(c) =>                (std::usize::MAX, c),
        AOMaxFrontierStateCap(mf,c) =>  (mf, c),
    };
    astar_opt_ex(s0, fnxt, fend, fheu, init_cap, max_frontier, std::usize::MAX)
}

///
pub fn astar<S,A,Fnxt,Fend,Fheu>(s0 : S, fnxt : Fnxt, fend : Fend, fheu : Fheu) -> Result<Vec<A>,AstarError<A>>
    where S : Ord, // State must implement Ord trait for BinaryHeap and BTreeSet
        A : Clone, // Actions are supposed to be lightweight, thus Clone
        Fnxt : Fn(&S) -> Vec<(A,S,f32)>, // Action generator function returns vector of triples (action, successor state, cost)
        Fend : Fn(&S) -> bool, // Destination test function returns true for goal states
        Fheu : Fn(&S) -> f32  { // Estimated cost of path from the state to a goal. Cost heuristic function should be admissible and consistent
    astar_opt(s0, fnxt, fend, fheu, AONone)
}

#[cfg(test)]
mod tests {
    use super::{astar, astar_opt, AstarOpt};

    #[derive(Debug,Clone,Copy)]
    struct GoDir(i32, i32);

    #[derive(Debug,PartialEq,Eq,PartialOrd,Ord)]
    struct Pos(i32,i32);

    fn nxt(s : &Pos) -> Vec<(GoDir,Pos,f32)> {
        let &Pos(x,y)=s;
        let mut ret=Vec::<(GoDir, Pos, f32)>::with_capacity(8);
        for &dir in [GoDir(1,0),GoDir(1,1),GoDir(0,1),GoDir(-1,1),GoDir(-1,0),GoDir(-1,-1),GoDir(0,-1),GoDir(1,-1)].iter() {
            let GoDir(dx,dy) = dir;
            let x1=x+dx;
            let y1=y+dy;
            ret.push((dir,Pos(x1,y1),f32::sqrt((dx*dx+dy*dy) as f32)));
        }
        ret
    }

    fn end(s : &Pos) -> bool {
        *s==Pos(5,10)
    }

    fn heur(s : &Pos) -> f32 {
        let x=(s.0 - 5) as f32;
        let y=(s.1 - 10) as f32;
        f32::sqrt(x*x+y*y)
    }

    #[test]
    fn not_very_good_test() {
        let path=astar(0i32, |s : &i32| vec![(*s,*s+1,0.0)], |s : &i32| *s==2, |_ : &i32| 0.0);
        println!("{:?}", path);
        assert_eq!(path, Ok(vec![0,1]));
        let path1=astar_opt(Pos(0,0), nxt, end, heur, AstarOpt::AOMaxFrontier(2));
        println!("{:?}", path1);  
        assert!(path1.is_err())
    }

    #[derive(Debug)]
    struct Maze {
        grid: Vec<Vec<bool>>, // true - passable, false - impassable
        tgt_x: usize,
        tgt_y: usize,
    }

    impl Maze {
        fn passable(&self, x: usize, y: usize) -> bool {
            self.grid[y][x]
        }
    }

    fn str2maze(s: &str) -> (Maze, usize, usize) {
        let mut grid = vec![];
        let mut tgt_x = 0;
        let mut tgt_y = 0;
        let mut start_x = 0;
        let mut start_y = 0;
        for (y,line) in s.lines().enumerate() {
            let mut gline = vec![];
            for (x,ch) in line.chars().enumerate() {
                match ch {
                    ' '|'.' => gline.push(true),
                    '*' => {
                        gline.push(true);
                        tgt_x = x;
                        tgt_y = y;
                    },
                    '@' => {
                        gline.push(true);
                        start_x = x;
                        start_y = y;
                    },
                    _ => {
                        gline.push(false);
                    },
                }
            }
            grid.push(gline);
        }
        (Maze { grid: grid, tgt_x: tgt_x, tgt_y: tgt_y }, start_x, start_y)
    }

    #[derive(Debug,PartialEq,Eq,PartialOrd,Ord)]
    struct MPos(usize,usize);

    #[derive(Debug,Clone,Copy,PartialEq,Eq)]
    enum MAction {
        Left, Right, Up, Down, Jump,
    }

    use self::MAction::{Left, Right, Up, Down, Jump};

    impl MAction {
        fn cost(self) -> f32 {
            1.
        }
    }

    struct MActions {
        cur: Option<MAction>,
    }

    fn actions() -> MActions {
        MActions{ cur: None }
    }

    impl Iterator for MActions {
        type Item = MAction;

        fn next(&mut self) -> Option<MAction> {
            match self.cur {
                None => {
                    self.cur = Some(Left);
                    self.cur
                },
                Some(Left) => {
                    self.cur = Some(Right);
                    self.cur
                }
                Some(Right) => {
                    self.cur = Some(Up);
                    self.cur
                }
                Some(Up) => {
                    self.cur = Some(Down);
                    self.cur
                }
                Some(Down) => {
                    self.cur = Some(Jump);
                    self.cur
                }
                Some(Jump) => {
                    self.cur = None;
                    self.cur
                }
            }
        }
    }

    fn mexec(&MPos(x,y): &MPos, a: MAction) -> MPos {
        match a {
            Left => MPos(x-1, y),
            Right => MPos(x+1, y),
            Up => MPos(x, y-1),
            Down => MPos(x, y+1),
            Jump => MPos(x, y),
        }
    }

    fn mnext(maze: &Maze, pos: &MPos) -> Vec<(MAction, MPos, f32)> {
        println!("Expanding {:?}", pos);
        actions()
            .filter_map(|a| {
                let next_pos =  mexec(pos, a);
                let MPos(x,y) = next_pos;
                println!("    with {:?}", a);
                if maze.passable(x,y) {
                    Some((a, next_pos, a.cost()))
                } else {
                    None
                }
            })
            .collect()
    }

    fn mheur(maze: &Maze, &MPos(x,y): &MPos) -> f32 {
        let tgt_x = maze.tgt_x as f32;
        let tgt_y = maze.tgt_y as f32;
        let x = x as f32;
        let y = y as f32;
        f32::sqrt((tgt_x-x)*(tgt_x-x)+(tgt_y-y)*(tgt_y-y))
    }

    fn mgoal(&Maze{tgt_x, tgt_y, ..}: &Maze, &MPos(x,y): &MPos) -> bool {
        tgt_x==x && tgt_y==y
    }


    #[test]
    fn maze1() {
        let (maze, sx, sy) = str2maze(
"
###############
#@.#.........##
##.###.#####.##
#..#.......#.##
##...#####.#.*#
###############
");
        assert!(sx==1);
        assert!(sy==2);
        assert!(maze.tgt_x==13);
        assert!(maze.tgt_y==5);
        let path = astar(MPos(sx,sy), |s|mnext(&maze,s), |s|mgoal(&maze,s), |s|mheur(&maze,s));
        println!("Path: {:?}", path);
        assert!(path==Ok(vec![Right,Down,Down,Down,Right,Right,Up,Right,Right,Up,Up,Right,Right,Right,Right,Right,Right,Down,Down,Down,Right]));
    }

    #[test]
    fn maze2() {
        let (maze, sx, sy) = str2maze(
"
###############
#@.#.........##
##.###.#####.##
#..#.......#.##
##...#####.#.*#
#.#.#..##.#####
#.#.#.###.#####
#.#.#.........#
#.....#######.#
###############
");
        assert!(sx==1);
        assert!(sy==2);
        assert!(maze.tgt_x==13);
        assert!(maze.tgt_y==5);
        let path = astar(MPos(sx,sy), |s|mnext(&maze,s), |s|mgoal(&maze,s), |s|mheur(&maze,s));
        println!("Path: {:?}", path);
        assert!(path==Ok(vec![Right,Down,Down,Down,Right,Right,Up,Right,Right,Up,Up,Right,Right,Right,Right,Right,Right,Down,Down,Down,Right]));
    }
}

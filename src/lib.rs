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

use AstarOpt::{AONone, AOMaxFrontier, AOStateCap, AOMaxFrontierStateCap};

pub enum AstarOpt {
    AONone,
    AOMaxFrontier(usize),
    AOStateCap(usize),
    AOMaxFrontierStateCap(usize,usize),
}

use AstarError::{NoPath, FrontierLimit};

#[derive(Debug,PartialEq,Eq)]
pub enum AstarError<A> {
    NoPath,
    FrontierLimit(Vec<A>),
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
    let state_arena=TypedArena::<S>::with_capacity(init_cap);
    let mut frontier=BinaryHeap::<Rc<FrontierElem<A,S>>>::new();
    let mut visited=BTreeSet::<&S>::new();
    let rs0 = state_arena.alloc(s0);
    frontier.push(Rc::new(FrontierElem::<A,S>{prev:None, action:None, state:rs0, cost:0.0, total_cost:0.0}));
    while let Some(fnode)=frontier.pop() {
        let cstate=(*fnode).state;
        if fend(cstate) {
            return Ok(collect_actions(&fnode));
        }
        if frontier.len()>max_frontier {
            return Err(FrontierLimit(collect_actions(&fnode)));
        }
        visited.insert(cstate);
        let nxts=fnxt(cstate);
        for (a,s,c) in nxts {
            if !visited.contains(&s) {
                let rs=state_arena.alloc(s);
                let hcost=fheu(rs);
                frontier.push(Rc::new(FrontierElem{prev: Some(fnode.clone()), action: Some(a), state: rs, cost:(*fnode).cost+c, total_cost:(*fnode).cost+c+hcost}));
            }
        }
    }
    return Err(NoPath);
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

    #[test]
    fn maze() {

    }
}

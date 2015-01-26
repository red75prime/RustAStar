extern crate arena;

use arena::TypedArena;
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

fn collect_actions<'a,A,S>(cur: &Rc<FrontierElem<'a,A,S>>, mut res: Vec<A>) -> Vec<A>
  where A:Clone {
  if let Some(ref action)=(**cur).action {
    res.push(action.clone());
  }
  match (**cur).prev {
    Some(ref prev) => collect_actions(prev, res),
    None => res,
  }
}

fn astar<S,A,Fnxt,Fend,Fheu>(s0 : S, fnxt : Fnxt, fend : Fend, fheu : Fheu) -> Result<Vec<A>,()>
  where S : Ord,
        A : Clone,
        Fnxt : Fn(&S) -> Vec<(A,S,f32)>,
        Fend : Fn(&S) -> bool,
        Fheu : Fn(&S) -> f32  {
  let mut frontier=BinaryHeap::<Rc<FrontierElem<A,S>>>::new();
  let mut visited=BTreeSet::<&S>::new();
  let state_arena=TypedArena::<S>::new();
  let rs0 = state_arena.alloc(s0);
  frontier.push(Rc::new(FrontierElem::<A,S>{prev:None, action:None, state:rs0, cost:0.0, total_cost:0.0}));
  while let Some(fnode)=frontier.pop() {
    let cstate=(*fnode).state;
    if fend(cstate) {
      return Ok(collect_actions(&fnode,vec![]));
    }
    visited.insert(cstate);
    for (a,s,c) in fnxt(cstate).drain() {
      if !visited.contains(&s) {
        let rs=state_arena.alloc(s);
        let hcost=fheu(rs);
        frontier.push(Rc::new(FrontierElem{prev: Some(fnode.clone()), action: Some(a), state: rs, cost:(*fnode).cost+c, total_cost:(*fnode).cost+c+hcost}));
      }
    }
  }
  return Err(());
}

mod tests {
  use super::astar;
  use std::num::Float as Fl;

  #[derive(Show,Clone)]
  struct GoDir(i32, i32);

  #[derive(PartialEq, PartialOrd, Eq, Ord)]
  struct Pos(i32,i32);

  fn nxt(s : &Pos) -> Vec<(GoDir,Pos,f32)> {
   let &Pos(x,y)=s;
   let mut ret=Vec::<(GoDir, Pos, f32)>::with_capacity(8);
   for dir in vec![GoDir(1,0),GoDir(1,1),GoDir(0,1),GoDir(-1,1),GoDir(-1,0),GoDir(-1,-1),GoDir(0,-1),GoDir(1,-1)].drain() {
     let dx=dir.0;
     let dy=dir.1;
     let x1=x+dx;
     let y1=y+dy;
     ret.push((dir,Pos(x1,y1),Fl::sqrt((dx*dx+dy*dy) as f32)));
   }
   ret
  }

  fn end(s : &Pos) -> bool {
    *s==Pos(5,10)
  }

  fn sqrt(x : f32) -> f32 {
   x.sqrt()
  }

  fn heur(s : &Pos) -> f32 {
    let x=(s.0-5) as f32;
    let y=(s.1-10) as f32;
    Fl::sqrt(x*x+y*y)
  }

  #[test]
  fn not_very_good_test() {
    let path=super::astar(0i32, |s : &i32| vec![(*s,*s+1,0.0)], |s : &i32| *s==2, |s : &i32| 0.0);
    println!("{:?}", path);
    assert_eq!(path, Ok(vec![0i32,1]));
    let path1=super::astar(Pos(0,0), nxt, end, heur);
    println!("{:?}", path1);  
  }
}

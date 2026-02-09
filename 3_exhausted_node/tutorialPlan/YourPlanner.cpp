#include "YourPlanner.h"
#include "RrtConConBase.h"
#include <rl/plan/SimpleModel.h>

YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  RrtConConBase::choose(chosen);
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  RrtConConBase::Vertex connected = RrtConConBase::connect(tree, nearest, chosen);

  if (connected == NULL)
  {
    tree[nearest.first].failureCount++;
  }
  
  return connected;
 
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
  //your modifications here
  return RrtConConBase::solve();
}

// Override nearest
RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());
  
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    if (tree[*i.first].failureCount > 10)
    {
      continue;
    }

    ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
    }
  }

  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}

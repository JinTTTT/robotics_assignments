#include "YourPlanner.h"
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
  // Initialize distances
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;

  // Define step sizes
  ::rl::math::Real largeDelta = this->delta;
  ::rl::math::Real smallDelta = this->delta * 0.1f; // 10% step size

  bool reached = false;

  // 'last' tracks the furthest valid configuration we have reached
  ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(*tree[nearest.first].q);
  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    // 1. Recalculate distance from the CURRENT 'last' position to the goal
    // FIX: Dereference 'last' (*last)
    distance = this->model->distance(*last, chosen);

    if (distance <= this->model->epsilon)
    {
      reached = true;
      break;
    }

    // 2. Try LARGE Step
    step = (::std::min)(distance, largeDelta);
    this->model->interpolate(*last, chosen, step / distance, next);
    this->model->setPosition(next);
    this->model->updateFrames();

    // FIX: Use isColliding(), not isInCollision()
    if (!this->model->isColliding())
    {
      // Large step valid! Update 'last'.
      *last = next;
    }
    else
    {
      // 3. Large Step Failed -> Try SMALL Step
      // We use the same 'distance' denominator because we are interpolating from the same 'last'
      step = (::std::min)(step, smallDelta);

      // If step is too tiny, we are just stuck.
      if (step <= this->model->epsilon)
      {
        break;
      }

      this->model->interpolate(*last, chosen, step / distance, next);
      this->model->setPosition(next);
      this->model->updateFrames();

      if (!this->model->isColliding())
      {
        // Small step valid! Update 'last'.
        // Note: The loop continues, so next iteration it will try Large Step again (resuming speed).
        *last = next;
      }
      else 
      {
        // Both Large and Small steps failed. We are blocked.
        break;
      }
    }
  }

  // --- ADD VERTEX LOGIC (Moved OUTSIDE the loop) ---
  
  // Only add a node if we actually moved away from the start
  if (this->model->distance(*tree[nearest.first].q, *last) > this->model->epsilon)
  {
    Vertex connected = this->addVertex(tree, last);
    this->addEdge(nearest.first, connected, tree);
    return connected;
  }

  return NULL;
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


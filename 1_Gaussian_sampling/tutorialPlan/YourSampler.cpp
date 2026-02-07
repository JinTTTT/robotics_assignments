#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            randEngine(::std::random_device()()),
            gaussDistribution(0, 1)
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            while (true)
            {   
            
                // Step 1: generate 1st sample uniformly
                ::rl::math::Vector q1(this->model->getDof());
                ::rl::math::Vector maximum(this->model->getMaximum());
                ::rl::math::Vector minimum(this->model->getMinimum());

                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    q1(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                }

                // Step 2: check if q1 in collision
                this->model->setPosition(q1);
                this->model->updateFrames();
                bool q1_collides = this->model->isColliding();

                // Step 3: generate 2nd sample Gaussian near q1
                ::rl::math::Vector q2(this->model->getDof());
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    ::rl::math::Real range = maximum(i) - minimum(i);
                    ::rl::math::Real sigma = 0.1 * range;
                    q2(i) = q1(i) + this->gaussDistribution(this->randEngine) * sigma;
                }
                this->model->clip(q2);

                // Step 4: check if q2 in collision
                this->model->setPosition(q2);
                this->model->updateFrames();
                bool q2_collides = this->model->isColliding();

                //Step 5: return the one is free and another is in collision
                if (q1_collides && !q2_collides)
                {
                    return q2;
                }
                else if (!q1_collides && q2_collides)
                {
                    return q1;
                }
                // If both are in collision or free, try again
                
            }
        }
        // ::rl::math::Vector
        // YourSampler::generate()
        // {
        //     // Our template code performs uniform sampling.
        //     // You are welcome to change any or all parts of the sampler.
        //     // BUT PLEASE MAKE SURE YOU CONFORM TO JOINT LIMITS,
        //     // AS SPECIFIED BY THE ROBOT MODEL!

        //     ::rl::math::Vector sampleq(this->model->getDof());

        //     ::rl::math::Vector maximum(this->model->getMaximum());
        //     ::rl::math::Vector minimum(this->model->getMinimum());

        //     for (::std::size_t i = 0; i < this->model->getDof(); ++i)
        //     {
        //         sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
        //     }

        //     // It is a good practice to generate samples in the
        //     // the allowed configuration space as done above.
        //     // Alternatively, to make sure generated joint 
        //     // configuration values are clipped to the robot model's 
        //     // joint limits, you may use the clip() function like this: 
        //     // this->model->clip(sampleq);

        //     return sampleq;
        // }

        ::std::uniform_real_distribution< ::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
        }
    }
}


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
            gaussDistribution(0, 1),
            bridgeRatio(1.0 / 5.0)  // rand < 1/5 use bridge, otherwise uniform
        {
        }

        YourSampler::~YourSampler()
        {
        }


        ::rl::math::Vector
        YourSampler::generate()
        {
            // Do uniform sampling (4 / 5 of the time)
            if (this->rand() > this->bridgeRatio)
            {
                ::rl::math::Vector qUniform(this->model->getDof());
                ::rl::math::Vector maximum(this->model->getMaximum());
                ::rl::math::Vector minimum(this->model->getMinimum());

                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    qUniform(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                }
                return qUniform;
                
            }
            else 
            {
                // Do bridge sampling (1 / 5 of the time
                while (true)
                {
                    // Step 1: generate q1 uniformly, q1 should be in collision
                    ::rl::math::Vector q1(this->model->getDof());
                    ::rl::math::Vector maximum(this->model->getMaximum());
                    ::rl::math::Vector minimum(this->model->getMinimum());

                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        q1(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                    }

                    // check if q1 in collision
                    this->model->setPosition(q1);
                    this->model->updateFrames();

                    if (!this->model->isColliding())
                    {
                        continue; // we need q1 to be in collision
                    }

                    // Step 2: generate q2 near q1 using gaussian, q2 should also in collision
                    ::rl::math::Vector q2(this->model->getDof());
                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        ::rl::math::Real range = maximum(i) - minimum(i);
                        // here we set sigma 0.1 for all joints, but need to tune later
                        ::rl::math::Real sigma = 0.1 * range;
                        q2(i) = q1(i) + this->gaussDistribution(this->randEngine) * sigma;
                    }
                    this->model->clip(q2);

                    // check if q2 in collision
                    this->model->setPosition(q2);
                    this->model->updateFrames();

                    if (!this->model->isColliding())
                    {
                        continue;
                    }

                    // Step 3: cal mid point of q1 and q2, midPoint should be free, then return
                    ::rl::math::Vector midPoint(this->model->getDof());
                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        midPoint(i) = (q1(i) + q2(i)) / 2;
                    }
                    
                    this->model->setPosition(midPoint);
                    this->model->updateFrames();

                    if (!this->model->isColliding())
                    {
                        return midPoint;
                    }                   
                }
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


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
            int max_attempts = 100;
            int attempt = 0;

            while (attempt < max_attempts) 
            {   
                attempt++;
                
                // Step 1: generate 1st sample uniformly
                ::rl::math::Vector q1(this->model->getDof());
                ::rl::math::Vector maximum(this->model->getMaximum());
                ::rl::math::Vector minimum(this->model->getMinimum());

                ::rl::math::Vector gaussian(this->model->getDof());
                ::rl::math::Vector sigma(this->model->getDof());

                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    q1(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                    gaussian(i) = this->gaussDistribution(this->randEngine);
                    sigma(i) = 2.0;

                }

                // // sigma has 6 dimensions, give joint 0 less var, joint 1 more var, joint 5 has most var
                // // since the change on base joint has more impact on the robot's configuration than the change on the end-effector joint
                // sigma(0) = 0.1;
                // sigma(1) = 0.1;
                // sigma(2) = 0.1;
                // sigma(3) = 0.3;
                // sigma(4) = 0.3;
                // sigma(5) = 0.3;

                // Step 2: check if q1 in collision
                this->model->setPosition(q1);
                this->model->updateFrames();
                bool q1_collides = this->model->isColliding();

                // Step 3: generate 2nd sample Gaussian near q1
                ::rl::math::Vector q2 = this->model->generatePositionGaussian(gaussian, q1, sigma);
                this->model->clip(q2);
                
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
                // If both are in collision or free, try again (loop continues)
            }
            
            // Fallback: if max_attempts reached, use uniform sampling
            ::rl::math::Vector sampleq(this->model->getDof());
            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
            }
            this->model->clip(sampleq);
            return sampleq;
        }
       
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


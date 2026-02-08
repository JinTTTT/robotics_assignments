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

            while (attempt < max_attempts)  // FIX: Changed from 'if' to 'while'
            {   
                attempt++;
                
                // Step 1: generate 1st sample uniformly
                ::rl::math::Vector q1(this->model->getDof());
                ::rl::math::Vector maximum(this->model->getMaximum());
                ::rl::math::Vector minimum(this->model->getMinimum());

                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    q1(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                }

                // Step 2: check if q1 in collision (with safety check)
                try {
                    this->model->setPosition(q1);
                    this->model->updateFrames();
                } catch (...) {
                    continue;  // Skip invalid configuration
                }
                bool q1_collides = this->model->isColliding();

                // Step 3: generate 2nd sample Gaussian near q1
                ::rl::math::Vector q2(this->model->getDof());
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    ::rl::math::Real range = maximum(i) - minimum(i);
                    
                    // FIX: Use adaptive sigma based on joint range
                    // Different joints have different ranges (5.59, 4.71, 4.71, 4.89, 3.49, 9.28 rad)
                    ::rl::math::Real sigma = 0.01 * range;  // 1% of range (~2-5 degrees)
                    
                    // Alternative options to try:
                    // ::rl::math::Real sigma = 0.02 * range;  // 2% of range (~4-10 degrees)
                    // ::rl::math::Real sigma = 0.005 * range; // 0.5% of range (~1-3 degrees)
                    
                    // SAFETY: Cap sigma to prevent extreme values
                    sigma = ::std::min(sigma, 0.1 * range);
                    
                    q2(i) = q1(i) + this->gaussDistribution(this->randEngine) * sigma;
                }
                
                // SAFETY: Clip to joint limits
                this->model->clip(q2);
                
                // SAFETY: Check for NaN/Inf values
                bool valid = true;
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    if (::std::isnan(q2(i)) || ::std::isinf(q2(i)))
                    {
                        valid = false;
                        break;
                    }
                }
                
                if (!valid)
                {
                    continue;  // Skip invalid configuration
                }

                // Step 4: check if q2 in collision (with safety check)
                try {
                    this->model->setPosition(q2);
                    this->model->updateFrames();
                } catch (...) {
                    continue;  // Skip invalid configuration
                }
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


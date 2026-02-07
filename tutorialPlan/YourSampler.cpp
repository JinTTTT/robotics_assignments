#include <chrono>
#include <cstddef>
#include <random>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"
#include <iostream>
namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            randEngine(::std::random_device()())
        {
        }

        YourSampler::~YourSampler()
        {
        }


        ::rl::math::Vector
        YourSampler::generate()
        {
            // Official RL library parameters: 5/6 uniform, 1/6 bridge
            const ::rl::math::Real bridgeRatio = 5.0 / 6.0;  // Uniform sampling ratio
            const ::rl::math::Real sigma = 0.1;
            const int maxBridgeAttempts = 100;  // Safety limit for bridge test

            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            // OFFICIAL APPROACH: Check if we should do UNIFORM (83%) or BRIDGE (17%)
            if (this->rand() > bridgeRatio)  // Only ~17% of the time: bridge test
            {
                // === BRIDGE TEST SAMPLING ===
                int attempt = 0;
                while (attempt < maxBridgeAttempts)
                {
                    attempt++;
                    
                    // STEP 1: Sample q1 uniformly
                    ::rl::math::Vector q1(this->model->getDof());
                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        q1(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                    }

                    this->model->setPosition(q1);
                    this->model->updateFrames();

                    // STEP 2: Check if q1 is in COLLISION (we need collision!)
                    if (!this->model->isColliding())
                    {
                        continue;
                    }

                    // STEP 3: Sample q2 using Gaussian around q1
                    ::rl::math::Vector q2(this->model->getDof());
                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        // Generate standard Gaussian N(0,1) then scale (like official)
                        ::rl::math::Real gaussSample = this->randGaussian(0.0, 1.0);
                        ::rl::math::Real jointRange = maximum(i) - minimum(i);
                        
                        // Add Gaussian noise to q1
                        q2(i) = q1(i) + gaussSample * sigma * jointRange;

                        // Clip to joint limits
                        q2(i) = (q2(i) < minimum(i)) ? minimum(i) : q2(i);
                        q2(i) = (q2(i) > maximum(i)) ? maximum(i) : q2(i);
                    }

                    this->model->setPosition(q2);
                    this->model->updateFrames();

                    // STEP 4: Check if q2 is in COLLISION (need both in collision!)
                    if (!this->model->isColliding())
                    {
                        continue;
                    }

                    // STEP 5: Compute midpoint using interpolate (like official)
                    ::rl::math::Vector midPoint(this->model->getDof());
                    this->model->interpolate(q1, q2, 0.5, midPoint);

                    this->model->setPosition(midPoint);
                    this->model->updateFrames();
                    
                    // STEP 6: Check if midpoint is FREE
                    if (!this->model->isColliding())
                    {
                        std::cout << "✓ Bridge sampling SUCCESS after " << attempt << " attempts" << std::endl;
                        return midPoint;
                    }
                }
                
                // Bridge test failed after maxAttempts, fall through to uniform
                std::cout << "✗ Bridge test failed after " << maxBridgeAttempts << " attempts - using uniform" << std::endl;
            }

            // === UNIFORM SAMPLING (83% of the time, OR bridge failed) ===
            ::rl::math::Vector uniSample(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                uniSample(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
            }

            return uniSample;
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

        ::rl::math::Real
        YourSampler::randGaussian(::rl::math::Real mean, ::rl::math::Real stddev)
        {
            ::std::normal_distribution<::rl::math::Real> normalDist(mean, stddev);
            return normalDist(this->randEngine);
        }
    }
}


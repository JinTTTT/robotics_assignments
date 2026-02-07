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
            // EXACT OFFICIAL RL LIBRARY IMPLEMENTATION
            // From: https://github.com/roboticslibrary/rl/blob/master/src/rl/plan/BridgeSampler.cpp
            
            const ::rl::math::Real ratio = static_cast<::rl::math::Real>(5) / static_cast<::rl::math::Real>(6);
            const ::rl::math::Real sigma = 0.1;
            
            // Official check: if rand() > ratio (5/6), do bridge test
            if (this->rand() > ratio)
            {
                // BRIDGE TEST - Exact official implementation
                ::rl::math::Vector q(this->model->getDof());
                ::rl::math::Vector gauss(this->model->getDof());
                
                while (true)  // Official uses infinite loop
                {
                    // Sample q2 (first collision point)
                    ::rl::math::Vector q2(this->model->getDof());
                    
                    // Generate uniform sample
                    ::rl::math::Vector maximum(this->model->getMaximum());
                    ::rl::math::Vector minimum(this->model->getMinimum());
                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        q2(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                    }
                    
                    this->model->setPosition(q2);
                    this->model->updateFrames();
                    
                    if (this->model->isColliding())
                    {
                        // Generate standard Gaussian samples N(0,1)
                        for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                        {
                            gauss(i) = this->randGaussian(0.0, 1.0);
                        }
                        
                        // Sample q3 (second collision point) using Gaussian around q2
                        ::rl::math::Vector q3(this->model->getDof());
                        for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                        {
                            ::rl::math::Real jointRange = maximum(i) - minimum(i);
                            q3(i) = q2(i) + gauss(i) * sigma * jointRange;
                            
                            // Clip to limits
                            if (q3(i) < minimum(i)) q3(i) = minimum(i);
                            if (q3(i) > maximum(i)) q3(i) = maximum(i);
                        }
                        
                        this->model->setPosition(q3);
                        this->model->updateFrames();
                        
                        if (this->model->isColliding())
                        {
                            // Both in collision, compute midpoint
                            this->model->interpolate(q2, q3, static_cast<::rl::math::Real>(0.5), q);
                            
                            this->model->setPosition(q);
                            this->model->updateFrames();
                            
                            if (!this->model->isColliding())
                            {
                                // Success! Midpoint is free
                                std::cout << "✓ Bridge sampling SUCCESS" << std::endl;
                                return q;
                            }
                        }
                    }
                }
            }
            else
            {
                // UNIFORM SAMPLING - Official fallback
                ::rl::math::Vector sampleq(this->model->getDof());
                ::rl::math::Vector maximum(this->model->getMaximum());
                ::rl::math::Vector minimum(this->model->getMinimum());
                
                for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                {
                    sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                }
                
                return sampleq;
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

        ::rl::math::Real
        YourSampler::randGaussian(::rl::math::Real mean, ::rl::math::Real stddev)
        {
            ::std::normal_distribution<::rl::math::Real> normalDist(mean, stddev);
            return normalDist(this->randEngine);
        }
    }
}


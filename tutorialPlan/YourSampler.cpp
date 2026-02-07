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
            const ::rl::math::Real bridgeRatio = 1.0 / 6.0;
            const ::rl::math::Real sigma = 0.1;

            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            if (this->rand() < bridgeRatio)
            {
                while (true)
                {
                    ::rl::math::Vector q1(this->model->getDof());
                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        q1(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                    }

                    this->model->setPosition(q1);
                    this->model->updateFrames();

                    if(!this->model->isColliding())
                    {
                        continue;
                    }

                    ::rl::math::Vector q2(this->model->getDof());
                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        
                        ::rl::math::Real jointRange = maximum(i) - minimum(i);
                        q2(i) = this->randGaussian(q1(i), sigma * jointRange);

                        q2(i) = (q2(i) < minimum(i)) ? minimum(i) : q2(i);
                        q2(i) = (q2(i) > maximum(i)) ? maximum(i) : q2(i);
                        
                    }

                    this->model->setPosition(q2);
                    this->model->updateFrames();

                    if(!this->model->isColliding())
                    {
                        continue;
                    }

                    ::rl::math::Vector midPoint(this->model->getDof());
                    this->model->interpolate(q1, q2, 0.5, midPoint)

                    this->model->setPosition(midPoint);
                    this->model->updateFrames();
                    
                    if(!this->model->isColliding())
                    {
                        std::cout << "Found valid bridge sample after " << (attempt+1) << " attempts" << std::endl;
                        return midPoint;
                    }

                }
            }
            

            // otherwise uniform sampling
            ::rl::math::Vector uniSample(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                std::cout << "No valid bridge sample, fallback to uniform sampling" << std::endl;
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


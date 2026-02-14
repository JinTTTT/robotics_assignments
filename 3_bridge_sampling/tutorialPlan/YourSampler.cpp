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
            bridgeRatio(1.0 / 6.0)  // rand < 1/6 use bridge, otherwise uniform
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
                    // prepare gaussian and sigma
                    ::rl::math::Vector gaussian(this->model->getDof());
                    ::rl::math::Vector sigma(this->model->getDof());
                    ::rl::math::Vector q1(this->model->getDof());
                    ::rl::math::Vector maximum(this->model->getMaximum());
                    ::rl::math::Vector minimum(this->model->getMinimum());

                    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                    {
                        q1(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
                        gaussian(i) = this->gaussDistribution(this->randEngine);
                        sigma(i) = 2.0;
                    }

                    // check if q1 in collision
                    this->model->setPosition(q1);
                    this->model->updateFrames();

                    if (!this->model->isColliding())
                    {
                        continue; // we need q1 to be in collision
                    }

                    // Step 2: generate q2 near q1 using gaussian, q2 should also in collision
                    ::rl::math::Vector q2 = this->model->generatePositionGaussian(gaussian, q1, sigma);
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


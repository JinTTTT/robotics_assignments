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
            randEngine(::std::random_device()()),
            gaussDistribution(0, 1),
            gaussEngine(::std::random_device()()),
            sigma(nullptr),
            bridgeTestRatio(static_cast<::rl::math::Real>(5) / static_cast<::rl::math::Real>(6))
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            if (this->rand() <= this->bridgeTestRatio)
            {
                return this->generateBridgeTest();
            }
            else
            {
                return this->generateUniform();
            }
        }

        ::rl::math::Vector
        YourSampler::generateUniform()
        {
            ::rl::math::Vector sampleq(this->model->getDof());
            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
            }

            return sampleq;
        }

        ::rl::math::Vector
        YourSampler::generateBridgeTest()
        {
            while (true)
            {
                ::rl::math::Vector q1 = this->generateUniform();
                
                this->model->setPosition(q1);
                this->model->updateFrames();
                
                if (this->model->isColliding())
                {
                    ::rl::math::Vector q2 = this->generateGaussianNear(q1);
                    
                    this->model->setPosition(q2);
                    this->model->updateFrames();
                    
                    if (this->model->isColliding())
                    {
                        ::rl::math::Vector midpoint(this->model->getDof());
                        this->model->interpolate(q1, q2, 0.5, midpoint);
                        
                        this->model->setPosition(midpoint);
                        this->model->updateFrames();
                        
                        if (!this->model->isColliding())
                        {
                            return midpoint;
                        }
                    }
                }
            }
        }

        ::rl::math::Vector
        YourSampler::generateGaussianNear(const ::rl::math::Vector& center)
        {
            ::rl::math::Vector result(this->model->getDof());
            
            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                ::rl::math::Real gaussianNoise = this->gauss();
                
                if (this->sigma != nullptr)
                {
                    result(i) = center(i) + gaussianNoise * (*this->sigma)(i);
                }
                else
                {
                    ::rl::math::Real range = this->model->getMaximum()(i) - this->model->getMinimum()(i);
                    ::rl::math::Real defaultSigma = 0.1 * range;
                    result(i) = center(i) + gaussianNoise * defaultSigma;
                }
            }
            
            this->model->clip(result);
            
            return result;
        }

        ::std::uniform_real_distribution< ::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        ::std::normal_distribution< ::rl::math::Real>::result_type
        YourSampler::gauss()
        {
            return this->gaussDistribution(this->gaussEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
            this->gaussEngine.seed(value);
        }

        void
        YourSampler::setSigma(::rl::math::Vector* sigma)
        {
            this->sigma = sigma;
        }

        ::rl::math::Vector*
        YourSampler::getSigma() const
        {
            return this->sigma;
        }

        void
        YourSampler::setBridgeTestRatio(const ::rl::math::Real& ratio)
        {
            this->bridgeTestRatio = ratio;
        }

        ::rl::math::Real
        YourSampler::getBridgeTestRatio() const
        {
            return this->bridgeTestRatio;
        }

    }
}


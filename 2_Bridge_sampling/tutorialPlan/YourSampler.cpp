#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            UniformSampler(),
            sigma(nullptr),
            gaussDistribution(0, 1),
            gaussEngine(::std::random_device()()),
            ratio(static_cast< ::rl::math::Real>(5) / static_cast< ::rl::math::Real>(6))
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::std::normal_distribution< ::rl::math::Real>::result_type
        YourSampler::gauss()
        {
            return this->gaussDistribution(this->gaussEngine);
        }

        ::rl::math::Vector
        YourSampler::generateCollisionFree()
        {
            if (this->rand() > this->ratio)
            {
                return this->generate();
            }
            else
            {
                ::rl::math::Vector gauss(this->model->getDof());
                ::rl::math::Vector pos(this->model->getDof());
                
                while (true)
                {
                    ::rl::math::Vector q1 = this->generate();
                    this->model->setPosition(q1);
                    this->model->updateFrames();

                    if (this->model->isColliding())
                    {
                        for (::std::size_t i = 0; i < this->model->getDof(); ++i)
                        {
                            gauss(i) = this->gauss();
                        }

                        ::rl::math::Vector q2 = this->model->generatePositionGaussian(gauss, q1, *this->sigma);

                        if (this->model->isColliding())
                        {
                            this->model->interpolate(q1, q2, static_cast< ::rl::math::Real>(0.5), pos);
                            this->model->setPosition(pos);
                            this->model->updateFrames();
                
                            if (!this->model->isColliding())
                            {
                                return pos;
                            }
                        }
                    }
                }
            }
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->gaussEngine.seed(value);
            this->randEngine.seed(value);
        }
    }
}

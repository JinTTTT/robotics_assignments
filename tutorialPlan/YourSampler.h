#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_

#include <rl/plan/UniformSampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        class YourSampler : public UniformSampler
        {
        public:
            YourSampler();
            virtual ~YourSampler();

            ::rl::math::Vector generateCollisionFree();
            virtual void seed(const ::std::mt19937::result_type& value);

            ::rl::math::Vector* sigma;
            ::rl::math::Real ratio;

        protected:
            ::std::normal_distribution< ::rl::math::Real>::result_type gauss();

            ::std::normal_distribution< ::rl::math::Real> gaussDistribution;
            ::std::mt19937 gaussEngine;

        private:
        };
    }
}

#endif // _YOURSAMPLER_H_

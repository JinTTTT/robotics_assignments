#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_

#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        class YourSampler : public Sampler
        {
        public:
            YourSampler();
            virtual ~YourSampler();

            ::rl::math::Vector generate();
            virtual void seed(const ::std::mt19937::result_type& value);

            void setSigma(::rl::math::Vector* sigma);
            ::rl::math::Vector* getSigma() const;

            void setBridgeTestRatio(const ::rl::math::Real& ratio);
            ::rl::math::Real getBridgeTestRatio() const;

        protected:
            ::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();
            ::std::normal_distribution< ::rl::math::Real>::result_type gauss();

            ::rl::math::Vector generateUniform();
            ::rl::math::Vector generateBridgeTest();
            ::rl::math::Vector generateGaussianNear(const ::rl::math::Vector& center);

            ::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
            ::std::mt19937 randEngine;

            ::std::normal_distribution< ::rl::math::Real> gaussDistribution;
            ::std::mt19937 gaussEngine;

            ::rl::math::Vector* sigma;
            ::rl::math::Real bridgeTestRatio;

        private:
        };
    }
}

#endif // _YOURSAMPLER_H_

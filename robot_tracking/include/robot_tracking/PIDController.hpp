#ifndef ROBOT_TRACKING__PIDCONTROLLER_HPP_
#define ROBOT_TRACKING__PIDCONTROLLER_HPP_

#include <cmath>

namespace robot_tracking
{
    class PIDController
    {
        public:
            PIDController(double min_ref, double max_ref, double min_output, double max_output);

            void set_pid(double n_KP, double n_KI, double n_KD);
            double get_output(double new_refrence);

        private:
            double KP_, KI_, KD_;

            double min_ref_, max_ref_;
            double min_output_, max_output_;
            double prev_error_, int_error_;
    }; //namespace robot_tracking
}

#endif 
#include "odom.hpp"
#include <cstdint>

pros::Imu inertial{UINT8_C(1)};

pros::Distance distanceFront{2};
pros::Distance distanceLeft{3};
pros::Distance distanceRight{4};

namespace pdi { 
    class PIDController {
    private:
        double Kp, Ki, Kd;
        double setpoint;
        double integral_sum;
        double last_error;
        double output_limit;
        double integral_limit;

    public:
        PIDController(double p, double i, double d, double limit, double i_limit) 
            :       
                Kp(p), 
                Ki(i), 
                Kd(d), 
                output_limit(limit), 
                integral_limit(i_limit), 
                integral_sum(0.0), last_error(0.0) 
        {}

        void set_setpoint(double target) {
            setpoint = target;
            integral_sum = 0.0; // reset integral sum when new target is set
            last_error = 0.0;
        }

        double calculate_output(double current_sensor_value) {
            // calc the error
            double error = setpoint - current_sensor_value;

            // calc the proportional term
            double proportional_term = Kp * error;

            // calc the integral term (with anti-windup)
            integral_sum += error;
            if (integral_sum > integral_limit) integral_sum = integral_limit;
            if (integral_sum < -integral_limit) integral_sum = -integral_limit;
            double integral_term = Ki * integral_sum;

            // calc the derivative term
            double derivative_term = Kd * (error - last_error);
            last_error = error;

            // calc the total output
            double output = proportional_term + integral_term + derivative_term;

            // apply output clamping (optional, but recommended)
            if (output > output_limit) output = output_limit;
            if (output < -output_limit) output = -output_limit;

            return output;
        }
    };
}

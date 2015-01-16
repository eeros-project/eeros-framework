#ifndef ORG_EEROS_CONTROL_MOUSEINPUT_HPP_
#define ORG_EEROS_CONTROL_MOUSEINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/Mouse.hpp>
#include <eeros/math/Matrix.hpp>


namespace eeros {
	namespace control {

        class MouseInput: public eeros::control::Block {
        public:
                MouseInput(std::string dev);
                virtual ~MouseInput();

                eeros::control::Output<double>& getOutX();
                eeros::control::Output<double>& getOutY();
                eeros::control::Output<double>& getOutZ();
                eeros::control::Output<double>& getOutR();
                eeros::control::Output<eeros::math::Matrix<4>>& getOut();

                virtual void run();

                virtual void setInitPos(double x, double y, double z, double r);
                virtual void setInitPos(eeros::math::Matrix<4> pos);
                virtual void reset(double x, double y, double z, double r);

                double axisScale_x = 0.0001;
                double axisScale_y = 0.0001;
                double axisScale_z = 0.001;
                double axisScale_r = 0.2;
                double min_x = -0.03;
                double max_x = 0.045;
                double min_y = -0.03;
                double max_y = 0.03;
                double min_z = -0.053;
                double max_z = -0.015;
                double min_r = -2.8;
                double max_r = 2.8;

                eeros::hal::Mouse j;

       protected:
                eeros::control::Output<double> outX;
                eeros::control::Output<double> outY;
                eeros::control::Output<double> outZ;
                eeros::control::Output<double> outR;
                eeros::control::Output<eeros::math::Matrix<4>> out;


                double x, y, z, r;
                bool first;
                std::thread* t;
        };
	};
};

#endif /* ORG_EEROS_CONTROL_MOUSEINPUT_HPP_ */

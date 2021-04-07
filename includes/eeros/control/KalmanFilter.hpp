#ifndef ORG_EEROS_CONTROL_KALMANFILTER_HPP_
#define ORG_EEROS_CONTROL_KALMANFILTER_HPP_

#include <eeros/core/System.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include "IndexOutOfBoundsFault.hpp"
#include <eeros/math/Matrix.hpp>
#include <mutex>

/**
 * Helper class to separate prediction and correction of the kalman filter.
 * This class is used to periodically run the prediction in a timedomain.
 * 
 * @param owner - pointer to the owner class
 */
template <uint8_t Nr_Of_Inputs, uint8_t Nr_Of_Outputs, uint8_t Nr_Of_States, uint8_t Nr_Of_Random_Variables>
class KalmanFilterPrediction;

/**
 * Helper class to separate prediction and correction of the kalman filter.
 * This class is used to periodically run the correction in a timedomain.
 * 
 * @param owner - pointer to the owner class
 */
template <uint8_t Nr_Of_Inputs, uint8_t Nr_Of_Outputs, uint8_t Nr_Of_States, uint8_t Nr_Of_Random_Variables>
class KalmanFilterCorrection;

/**
 * A kalman filter block is used to estimate the state of a system. 
 * The system must be described in a state space representation:
 * 
 * u -----------------------> physical system ------------------------------> y
 *     |                                                              |
 *     |                                                              |
 *     |   ---------------------> D ---------------------             |
 *     |  |                                              |            |
 *     |  |                                              | +          | -
 *     |  |         +      x(k-1)          x(k)       +  v   y_hat +  v
 *      -----> Bd ---> 0 ----------> z⁻1 -------> C ---> 0 ---------> 0
 *                     ^                                 |            |
 *                   + |                                 | +          |
 *                     |                                 v            |
 *                      -------------- Ad <------------- 0 <--- K ----
 *                                                          +
 *
 * with x - state vector
 *      u - input vector of the physical system
 *      y - output vector of the physical system
 *      
 *      Ad - time discrete system matrix
 *      Bd - time discrete input matrix
 *      C - output matrix
 *      D - feed forward matrix
 *      K - kalman feedback matrix
 * 
 * Furthermore, disturbances in the system need to be modeled as noise in a separate 
 * vector z with a related kovariance matrix G. Measurement noise is added to the
 * output by introducing a vector v.
 * 
 * So the complete state space model gets
 *          x(k-1) = Ad*x(k) + Bd*u(k) + Gd*z(k)
 *          y(k) = C*x(k) + D*u(k) + v(k)
 * 
 * For a detailed introduction to the topic, please refer to the book 
 * "Kalman-Filter Einführung in die Zustandsschätzung und ihre Anwendungen für 
 * eingebettete Systeme" by Reiner Marchthaler and Sebastian Dingler.
 * 
 * The kalman filter block has two separate blocks included. One is for the 
 * prediction and the other one for the correction. The correction block
 * should be run after reading the sensor values, while the prediction block should 
 * run after the input vector is defined. The two blocks can run in different time domains.
 * 
 * @tparam Nr_Of_Inputs - number of system inputs
 * @tparam Nr_Of_Outputs - number of system outputs
 * @tparam Nr_Of_States - number of states
 * @tparam Nr_Of_Random_Variables - number of random variables to model the system noise
 * @since v1.2
 */
template <uint8_t Nr_Of_Inputs, uint8_t Nr_Of_Outputs, uint8_t Nr_Of_States, uint8_t Nr_Of_Random_Variables>
class KalmanFilter : public eeros::control::Block
{
public:
    /**
     * Constructs a kalman filter instance by providing the matrices Ad, Bd, C, Gd, Q and R.
     * The matrix D is set to 0 (no feed forward path).
     * The initial state of the system x is set to 0.
     * The initial covariance of the estimation error is set to 0.
     *
     * @param Ad - time discrete system matrix
     * @param Bd - time discrete input matrix
     * @param C - output matrix
     * @param Gd - time discrete kovariance matrix of the system disturbance
     * @param Q - variance of the system noise
     * @param R - variance of the process noise
     */
    KalmanFilter(eeros::math::Matrix<Nr_Of_States, Nr_Of_States> Ad,
                 eeros::math::Matrix<Nr_Of_States, Nr_Of_Inputs> Bd,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_States> C,
                 eeros::math::Matrix<Nr_Of_States, Nr_Of_Random_Variables> Gd,
                 eeros::math::Matrix<Nr_Of_Random_Variables, Nr_Of_Random_Variables> Q,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_Outputs> R)
        : Ad(Ad), Bd(Bd), C(C), Gd(Gd), Q(Q), R(R), predict(this), correct(this)
    {
        this->D.zero();
        this->x.zero();
        this->P.eye();
        this->eye.eye();
        this->GdQGdT = Gd * Q * Gd.transpose();
    }
    /**
     * Constructs a kalman filter instance by providing the matrices Ad, Bd, C, D, Gd, Q and R.
     * The initial state of the system x is set to 0.
     * The initial covariance of the estimation error is set to 0.
     *
     * @param Ad - time discrete system matrix
     * @param Bd - time discrete input matrix
     * @param C - output matrix
     * @param D - feed forward matrix
     * @param Gd - time discrete kovariance matrix of the system disturbance
     * @param Q - variance of the system noise
     * @param R - variance of the process noise
     */
    KalmanFilter(eeros::math::Matrix<Nr_Of_States, Nr_Of_States> Ad,
                 eeros::math::Matrix<Nr_Of_States, Nr_Of_Inputs> Bd,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_States> C,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_Inputs> D,
                 eeros::math::Matrix<Nr_Of_States, Nr_Of_Random_Variables> Gd,
                 eeros::math::Matrix<Nr_Of_Random_Variables, Nr_Of_Random_Variables> Q,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_Outputs> R)
        : Ad(Ad), Bd(Bd), C(C), D(D), Gd(Gd), Q(Q), R(R), predict(this), correct(this)
    {
        this->x.zero();
        this->P.eye();
        this->eye.eye();
        this->GdQGdT = Gd * Q * Gd.transpose();
    }
    /**
     * Constructs a kalman filter instance by providing the matrices Ad, Bd, C, D, Gd, Q, R and P and the vector x.
     *
     * @param Ad - time discrete system matrix
     * @param Bd - time discrete input matrix
     * @param C - output matrix
     * @param D - feed forward matrix
     * @param Gd - time discrete kovariance matrix of the system disturbance
     * @param Q - variance of the system noise
     * @param R - variance of the process noise
     * @param P - initial covariance of the estimation error
     * @param x - initial state of the system
     */
    KalmanFilter(eeros::math::Matrix<Nr_Of_States, Nr_Of_States> Ad,
                 eeros::math::Matrix<Nr_Of_States, Nr_Of_Inputs> Bd,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_States> C,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_Inputs> D,
                 eeros::math::Matrix<Nr_Of_States, Nr_Of_Random_Variables> Gd,
                 eeros::math::Matrix<Nr_Of_Random_Variables, Nr_Of_Random_Variables> Q,
                 eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_Outputs> R,
                 eeros::math::Matrix<Nr_Of_States, Nr_Of_States> P,
                 eeros::math::Vector<Nr_Of_States> x)
        : Ad(Ad), Bd(Bd), C(C), D(D), Gd(Gd), Q(Q), R(R), P(P), x(x), predict(this), correct(this)
    {
        this->eye.eye();
        this->GdQGdT = Gd * Q * Gd.transpose();
    }

    /**
     * Get vector element with index index of system input vector u.
     * 
     * @param index - element index
     */
    eeros::control::Input<double> &getU(uint8_t index)
    {
        if (index < 0 || index >= Nr_Of_Inputs)
        {
            throw eeros::control::IndexOutOfBoundsFault("Trying to get inexistent element of system input vector u in Block " +
                                                        this->getName() + ".");
        }
        return u.getIn(index);
    }

    /**
     * Get vector element with index index of system output vector y.
     * 
     * @param index - element index
     */
    eeros::control::Input<double> &getY(uint8_t index)
    {
        if (index < 0 || index >= Nr_Of_Outputs)
        {
            throw eeros::control::IndexOutOfBoundsFault("Trying to get inexistent element of system output vector y in Block " +
                                                        this->getName() + ".");
        }
        return y.getIn(index);
    }

    /**
     * Get vector element with index index of system state vector x.
     * 
     * @param index - element index
     */
    eeros::control::Output<double> &getX(uint8_t index)
    {
        if (index < 0 || index >= Nr_Of_States)
        {
            throw eeros::control::IndexOutOfBoundsFault("Trying to get inexistent element of system state vector x in Block " +
                                                        this->getName() + ".");
        }
        return out[index];
    }

    /**
     * Predict current system state
     */
    void prediction()
    {
        std::lock_guard<std::mutex> lock(mtx);
        u.run();
        x = Ad * x + Bd * u.getOut().getSignal().getValue();
        for (uint8_t i = 0; i < Nr_Of_States; i++)
        {
            out[i].getSignal().setValue(x[i]);
            out[i].getSignal().setTimestamp(eeros::System::getTimeNs());
        }
        P = Ad * P * Ad.transpose() + GdQGdT;
    }

    /**
     * Correct current system state
     */
    void correction()
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (first)
        {
            for (uint8_t i = 0; i < Nr_Of_States; i++)
            {
                out[i].getSignal().setValue(x[i]);
                out[i].getSignal().setTimestamp(eeros::System::getTimeNs());
            }
            first = false;
        }
        else
        {
            y.run();
            u.run();
            CPCTR = (C * P * C.transpose() + R);
            K = P * C.transpose() * !CPCTR;
            dy = y.getOut().getSignal().getValue() - C * x - D * u.getOut().getSignal().getValue();
            x = x + K * dy;
            for (uint8_t i = 0; i < Nr_Of_States; i++)
            {
                out[i].getSignal().setValue(x[i]);
                out[i].getSignal().setTimestamp(eeros::System::getTimeNs());
            }
            P = (eye - K * C) * P;
        }
    }

    KalmanFilterPrediction<Nr_Of_Inputs, Nr_Of_Outputs, Nr_Of_States, Nr_Of_Random_Variables> predict;
    KalmanFilterCorrection<Nr_Of_Inputs, Nr_Of_Outputs, Nr_Of_States, Nr_Of_Random_Variables> correct;

protected:
    std::mutex mtx;
    eeros::math::Vector<Nr_Of_States> x;
    eeros::math::Vector<Nr_Of_Outputs> dy;
    eeros::control::Mux<Nr_Of_Outputs> y;
    eeros::control::Mux<Nr_Of_Inputs> u;
    eeros::control::Output<double> out[Nr_Of_States];
    eeros::math::Matrix<Nr_Of_States, Nr_Of_States> Ad, P, eye, GdQGdT;
    eeros::math::Matrix<Nr_Of_States, Nr_Of_Inputs> Bd;
    eeros::math::Matrix<Nr_Of_States, Nr_Of_Outputs> K;
    eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_States> C;
    eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_Inputs> D;
    eeros::math::Matrix<Nr_Of_States, Nr_Of_Random_Variables> Gd;
    eeros::math::Matrix<Nr_Of_Random_Variables, Nr_Of_Random_Variables> Q;
    eeros::math::Matrix<Nr_Of_Outputs, Nr_Of_Outputs> R, CPCTR;
    bool first = true;

private:
    /**
     * This method is required because the klass inherits from eeros::control::Block,
     * which inherits from eeros::Runnable.
     * This run method does not do anything. Use the run methods of the helper classes instead.
     */
    void run(){};
};

template <uint8_t Nr_Of_Inputs, uint8_t Nr_Of_Outputs, uint8_t Nr_Of_States, uint8_t Nr_Of_Random_Variables>
class KalmanFilterPrediction : public eeros::control::Block
{
public:
    KalmanFilterPrediction(KalmanFilter<Nr_Of_Inputs, Nr_Of_Outputs, Nr_Of_States, Nr_Of_Random_Variables> *owner)
        : owner(owner) {}

    virtual void run()
    {
        owner->prediction();
    }

private:
    KalmanFilter<Nr_Of_Inputs, Nr_Of_Outputs, Nr_Of_States, Nr_Of_Random_Variables> *owner;
};

template <uint8_t Nr_Of_Inputs, uint8_t Nr_Of_Outputs, uint8_t Nr_Of_States, uint8_t Nr_Of_Random_Variables>
class KalmanFilterCorrection : public eeros::control::Block
{
public:
    KalmanFilterCorrection(KalmanFilter<Nr_Of_Inputs, Nr_Of_Outputs, Nr_Of_States, Nr_Of_Random_Variables> *owner)
        : owner(owner) {}

    virtual void run()
    {
        owner->correction();
    }

private:
    KalmanFilter<Nr_Of_Inputs, Nr_Of_Outputs, Nr_Of_States, Nr_Of_Random_Variables> *owner;
};

#endif // ORG_EEROS_CONTROL_KALMANFILTER_HPP_

#ifndef EMBEDDED_PID_HPP
#define EMBEDDED_PID_HPP

#include "infra/util/Optional.hpp"
#include <chrono>
#include <cstdlib>

namespace control_toolbox
{
    template<class T>
    class Pid;

    using PidI32 = Pid<int32_t>;
    using PidFloat = Pid<float>;

    template<class T>
    class Pid
    {
    public:
        template<class T>
        struct Tunnings
        {
            T kp;
            T ki;
            T kd;
        };

        template<T>
        struct Limits
        {
            T min;
            T max;
        };

        Pid(Tunnings<T> tunnings, std::chrono::microseconds sampleTime, Limits<T> limits, bool autoMode = true, bool proportionalOnMeasurement = false, bool differentialOnMeasurement = true);

        void SetPoint(T setPoint);

        T Process(T& measuredProcessVariable);

        void Enable();
        void Disable();

        void SetLimits(Limits<T> limits);
        void SetTunnings(Tunnings<T> tunnings);
        void SetSampleTime(std::chrono::microseconds sampleTime);

        void Reset();

    private:
        T Clamp(T& input);

    private:
        Tunnings<T> tunnings;
        std::chrono::microseconds sampleTime;
        Limits<T> limits; 
        bool autoMode;
        bool proportionalOnMeasurement;
        bool differentialOnMeasurement;

        infra::Optional<T> setPoint = 0;
        T proportional = 0;
        T integral = 0;
        T derivative = 0;

        infra::Optional<T> lastOutput = infra::none;
        infra::Optional<T> lastError = infra::none;
        infra::Optional<T> lastInput = infra::none;
    };

    ////    Implementation    ////

    template<class T>
    Pid<T>::Pid(Tunnings<T> tunnings, infra::Duration sampleTime, Limits<T> limits, bool autoMode, bool proportionalOnMeasurement, bool differentialOnMeasurement)
        : tunnings(tunnings)
        , sampleTime(sampleTime)
        , limits(limits)
        , autoMode(autoMode)
        , proportionalOnMeasurement(proportionalOnMeasurement)
        , differentialOnMeasurement(differentialOnMeasurement)
    {
        integral = Clamp();
    }

    template<class T>
    T Pid<T>::SetPoint(T setPoint)
    {
        setPoint.Emplace(setPoint);
    }

    template<class T>
    T Pid<T>::Clamp(T& input)
    {
        if (input > limits.max)
            return limits.max;

        if (input < limits.min)
            return limits.min;

        return input;
    }

    template<class T>
    T Pid<T>::Process(T& measuredProcessVariable)
    {
        if (!setPoint || !autoMode)
            return lastOutput.ValueOr(*setPoint);

        T error = setPoint - measuredProcessVariable;
        T derivativeInput = measuredProcessVariable - lastInput.ValueOr(measuredProcessVariable);
        T derivativeError = error - lastError.ValueOr(error);

        CalculateProportional(error, derivativeInput);
        CalculateIntegral(error);
        CalculateDerivative(derivativeInput, derivativeError);

        T output = Clamp(proportional + integral + derivative);

        UpdateLasts(output, error, measuredProcessVariable);

        return output;
    }

    template<class T>
    void Pid<T>::CalculateProportional(T& error, T& derivativeInput)
    {
        if (proportionalOnMeasurement)
            proportional = tunnings.kp * error;
        else
            proportional -= tunnings.kp * derivativeInput;
    }

    template<class T>
    void Pid<T>::CalculateIntegral(T& error)
    {
        integral += tunnings.ki * error * static_cast<T>(std::chrono::duration_cast<std::chrono::microseconds>(sampleTime).count());
        integral = Clamp(integral);
    }

    template<class T>
    void Pid<T>::CalculateDerivative(T& derivativeInput, T& derivativeError)
    {
        if (differentialOnMeasurement)
            derivative = -tunnings.kd * derivativeInput / static_cast<T>(std::chrono::duration_cast<std::chrono::microseconds>(sampleTime).count());
        else
            derivative -= tunnings.kd * derivativeError / static_cast<T>(std::chrono::duration_cast<std::chrono::microseconds>(sampleTime).count());
    }
}

#endif

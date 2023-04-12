#include <cmath>
#include <algorithm>

#include "pid.h"

PID::PID(double* in, double* out, double min, double max, double setpoint ,double kp, double ki, double kd, double r)
{
    m_in = in;
    m_out = out;
    m_min = min;
    m_max = max;
    m_r = r;
    m_setpoint = setpoint;
    setValue(ki, kp, kd);
}

void PID::calculate()
{
    double err = m_setpoint - *m_in;
    int_err += err;
    // low pass filter to filter out high frequencies noise from the sensor
    double deri_err = (1-m_r)*err - m_r*prev_err;
    //PID with conditional integration
    *m_out = m_kp*err + flag_int * m_ki*int_err + m_kd*deri_err;
    // Anti windup
    flag_int = !(ioHasSameSign() && isSaturated());
    clampOutputLimit();

    prev_err = err;
}

void PID::setValue(double ki, double kp, double kd)
{
    m_ki = ki;
    m_kp = kp;
    m_kd = kd;
}

void PID::setLPFilterGain(double r)
{
    m_r = r;
}

void PID::clampOutputLimit()
{
    std::clamp(*m_out, m_min, m_max);
}

bool PID::ioHasSameSign()
{
    return std::signbit(*m_out) == std::signbit(*m_in);
}

bool PID::isSaturated()
{
    return (*m_out < m_min || *m_out > m_max);
}

double PID::getKp()
{
    return m_kp;
}

double PID::getKd()
{
    return m_kd;
}

double PID::getKi()
{
    return m_ki;
}

double PID::getLPFilterGain()
{
    return m_r;
}

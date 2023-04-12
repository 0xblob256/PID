#ifndef PID_H
#define PID_H
class PID
{
public:
    PID(double* in, double* out, double min, double max, double setpoint ,double kp, double ki, double kd, double r);

    void setValue(double ki, double kp, double kd);
    void setLPFilterGain(double r);
    double getKp();
    double getKi();
    double getKd();
    double getLPFilterGain();
    void calculate();
private:
    //void initialise();

    bool isSaturated();
    void clampOutputLimit();
    bool ioHasSameSign();

    double* m_in;
    double* m_out;

    double m_setpoint;
    double m_min;
    double m_max;
    //lp filter gain
    double m_r;

    double m_ki;
    double m_kp;
    double m_kd;

    double int_err;
    double prev_err;
    bool flag_int;
};
#endif // PID_H

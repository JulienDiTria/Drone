#ifndef PID_vDITRIA
#define PID_vDITRIA

class PID
{
    public:
    PID(double*, double*, double*, double, double, double); // * constructor. Input, Output, and Setpoint, kp, ki, kd
    bool Compute();                       // * performs the PID calculation.
    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default
    void SetTunings(double, double, double);

    //Display functions ****************************************************************
    double GetKp();						  // These functions query the pid for interal values.
    double GetKi();						  // they were created mainly for the pid front-end,
    double GetKd();						  // where it's important to know what is actually

    private:
    void Initialize();

    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    double *myInput;            // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;           //   This creates a hard link between the variables and the
    double *mySetpoint;         //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.

    unsigned long prevTime;
    double prevError;
    double P_term, I_term, D_term;
    double output;
    double outMin, outMax;
};
#endif

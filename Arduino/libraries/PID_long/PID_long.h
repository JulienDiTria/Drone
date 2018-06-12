#ifndef PID_LONG
#define PID_LONG

class PID
{
    public:
    PID(long*, long*, long*, long, long, long); // * constructor. Input, Output, and Setpoint, kp, ki, kd
    bool Compute();                       // * performs the PID calculation.
    void SetOutputLimits(long, long); // * clamps the output to a specific range. 0-255 by default
    void SetTunings(long, long, long);
	void Start();

    long GetKp();						  // These functions query the pid for internal values.
    long GetKi();						  // they were created mainly for the pid front-end,
    long GetKd();						  // where it's important to know what is actually
	
	long GetDT();

    private:
    void Initialize();

    long kp;                  // * (P)roportional Tuning Parameter
    long ki;                  // * (I)ntegral Tuning Parameter
    long kd;                  // * (D)erivative Tuning Parameter

    long *myInput;            // * Pointers to the Input, Output, and Setpoint variables
    long *myOutput;           //   This creates a hard link between the variables and the
    long *mySetpoint;         //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.

    long prevTime, dt;
    long prevError;
    long P_term, I_term, D_term;
    long output;
    long outMin, outMax;
};
#endif

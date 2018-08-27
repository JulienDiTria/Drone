/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_vDITRIA.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    PID::SetTunings(Kp, Ki, Kd);
    PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits
    I_term=0;
    prevTime = millis();
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
    unsigned long now = millis(); // in msec
    double dt = ((double)(now - prevTime))/1000; // in sec
    /*Compute all the working error variables*/
    double input = *myInput;
    double error = *mySetpoint - input;
    double dError = error-prevError;

    // Integral term
    I_term+=ki*error*dt;
    if(I_term > outMax) I_term = outMax;
    else if(I_term < outMin) I_term = outMin;

    // Proportional term*/
    P_term = kp * error;

    // Derivative term
    D_term = kd * dError;

    output=P_term+I_term+D_term;
    if(output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
    *myOutput = output;

    /*Remember some variables for next time*/
    prevError = error;
    prevTime = now;
    return true;
}


void PID::SetTunings(double Kp, double Ki, double Kd){
    if (Kp<0 || Ki<0 || Kd<0) return;
    kp = Kp; ki = Ki; kd = Kd;
}

void PID::SetOutputLimits(double Min, double Max)
{
   Serial.println("PID::SetOutputLimits");
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  kp;}
double PID::GetKi(){ return  ki;}
double PID::GetKd(){ return  kd;}


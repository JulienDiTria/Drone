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

#include <PID_long.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(long* Input, long* Output, long* Setpoint,
        long Kp, long Ki, long Kd)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    PID::SetTunings(Kp, Ki, Kd);
    PID::SetOutputLimits(-6000, 6000);		//default output limit					
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
    long now = millis(); // in ms
    dt = (now - prevTime); // in ms
    /*Compute all the working error variables*/
    long input = *myInput;
    long error = *mySetpoint - input;
    long dError = error-prevError;

    // Integral term
    I_term+=((ki*error*dt)/1000);
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


void PID::SetTunings(long Kp, long Ki, long Kd){
    if (Kp<0 || Ki<0 || Kd<0) return;
    kp = Kp; ki = Ki; kd = Kd;
}

void PID::SetOutputLimits(long Min, long Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
}

void PID::Start(){
	I_term = 0;
	prevTime = millis();
}

long PID::GetKp(){ return  kp;}
long PID::GetKi(){ return  ki;}
long PID::GetKd(){ return  kd;}
long PID::GetDT(){ return  dt;}


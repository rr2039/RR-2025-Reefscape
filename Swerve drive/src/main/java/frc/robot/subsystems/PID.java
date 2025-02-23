package frc.robot.subsystems;
import java.lang.Math;
/*
 * This Mini PID library is made to control the position of motors, through set(double speed) function as
 * part of rev library. set(double speed) accepts values from -1 to 1.
 */

public class PID {

    //defult
    double Kp = 0;
    double Ki = 0;
    double KiClamp = 0; //clamps ki to prevent saturation
    double Kd = 0;;
    double kMaxAcceleration = 0;
    double kMaxVelocity = 1;

    //actively used
    double input = 0;
    double setpoint = 0;
    double oldError = 0;
    double accumulatedError = 0;
    double oldPidOut = 0;
    double pidOut = 0;
    

    //Feedforward
    //COMING SOON

    //constructer when new PID controller created
    public PID(double kp, double ki, double kd, double kiclamp, double kmaxvelocity,double kmaxacceleration){
        Kp = kp;
        Ki = ki;
        KiClamp = kiclamp;
        Kd = kd;
        if (kmaxvelocity > 0) kMaxVelocity = kmaxvelocity;
        kMaxAcceleration = kmaxacceleration;

    }

    //should be run perdiodicaly at constant rate for best results
    public void calculatePID(){
        //some math
        double error = setpoint - input;
        accumulatedError += error;
        accumulatedError = Math.min(Math.max(accumulatedError,-KiClamp),KiClamp);
        double delta_error = error-oldError;
                
        pidOut = Kp*error+Ki*accumulatedError+Kd*delta_error; //main pid math
        pidOut = Math.min(Math.max(pidOut,-kMaxVelocity),kMaxVelocity); //final result is clamped, which is basicaly velocity limit, voltage and speed are proportional
        
        if (kMaxAcceleration > 0) pidOut = Math.min(Math.max(oldPidOut-pidOut,kMaxAcceleration),kMaxAcceleration); //limits acceleration, takes derivitive of pidout and limits

        oldError = error; //update var for next cycle
    }
    
    public double getPID(){
        return pidOut;

    }

    public void setNewPoint(double setPoint)
    {
        setpoint = setPoint;
    }
}



package frc.robot.modules;

//WPILib
import edu.wpi.first.wpilibj.Timer;

public class MotorRampUp {
    public static double deltaTime;

    /**
    * Uses -1/x as a parent function to make a nice ramp-up curve on the motor output. 
    * Includes the relationship that the time taken to ramp-up to within 95% of the desired speed
    * in seconds can be included in the function by a multiplier, according to the function 1/(time) * 100
    * If timeOrigin jumps to zero, the motor starts going backwards, thus the ternary.

    * @param time Desired time to get up to speed
    * @param timeorigin Uptime at the time of start
    * @param setpoint Motor speed
    * @return Value of the motor speed at the time of interpolation
    */
    public static double RampUp(double time, double timeorigin, double setpoint){
        double multiplier = 100 / time;
        double offset = 1 / multiplier;
        deltaTime = Timer.getFPGATimestamp() - timeorigin;
        if(setpoint != 0){
            return (-Math.signum(setpoint)) * (1 / (multiplier * (deltaTime+offset))) + (setpoint);
        }else{
            return 0;
        }
    }
}

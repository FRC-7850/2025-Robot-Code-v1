package frc.robot.subsystems;

//WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

//Rev
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

    //File Structure
//Modules
import frc.robot.modules.MotorRampUp;
//Constants
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ClimbSubsystem extends SubsystemBase{
    private static final SparkMax climbMotor = new SparkMax(CanIDConstants.kClimberCanId, MotorType.kBrushless);
    public static double outputSetpoint;
    public static double timeOrigin;
    public static double motorZeroPos;

    public static RelativeEncoder motorEncoder = climbMotor.getEncoder();

    public ClimbSubsystem(){
        motorZeroPos = motorEncoder.getPosition();
    }

    public static void Climb(double speed){
        outputSetpoint = speed;
        // timeSetpoint = Timer.getFPGATimestamp() + NeoMotorConstants.kClimberTimeToInterpolate;
        timeOrigin = Timer.getFPGATimestamp();
    }

    public static void StopClimb(){
        outputSetpoint = 0;
    }

    public static void Safety(){

    }

    @Override
    public void periodic(){
        //Protect from driving into bellypan
        if(motorEncoder.getPosition() >= motorZeroPos && outputSetpoint > 1){
            climbMotor.set(MotorRampUp.RampUp(NeoMotorConstants.kClimberTimeToInterpolate, timeOrigin, outputSetpoint));
        }

    }
}

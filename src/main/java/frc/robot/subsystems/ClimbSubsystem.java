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
import frc.robot.Constants.OIConstants;

public class ClimbSubsystem extends SubsystemBase{
    private static final SparkMax climbMotor = new SparkMax(OIConstants.kClimberCanId, MotorType.kBrushless);

    public static RelativeEncoder motorEncoder = climbMotor.getEncoder();

    public ClimbSubsystem(){
    }

    public static void Climb(double polarity){
        climbMotor.set(polarity * 0.2);
    }

    public static void Safety(){

    }

    @Override
    public void periodic(){
    }
}
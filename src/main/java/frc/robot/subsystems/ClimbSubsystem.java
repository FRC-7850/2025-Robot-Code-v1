package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Rev
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

    //File Structure
//Modules
//Constants
import frc.robot.Constants.OIConstants;

public class ClimbSubsystem extends SubsystemBase{
         ShuffleboardTab climberTestingTab = Shuffleboard.getTab("ClimbTestingTab");
         GenericEntry Id50Speed;
         GenericEntry Id51Speed;
    private static final SparkMax climbMotor = new SparkMax(OIConstants.kClimberCanId, MotorType.kBrushless);
    private static final SparkMax climbFollower = new SparkMax(51, MotorType.kBrushless);
    public static RelativeEncoder motorEncoder = climbMotor.getEncoder();

    public ClimbSubsystem(){
        Id50Speed = climberTestingTab.add("Id 50", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        Id51Speed = climberTestingTab.add("Id 51", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    }

    public static void Climb(double polarity){
        climbMotor.set(polarity);
    }

    public static void Safety(){

    }

    @Override
    public void periodic(){
        Id50Speed.setDouble(climbMotor.getAppliedOutput());
        Id51Speed.setDouble(climbMotor.getAppliedOutput());
    }
}
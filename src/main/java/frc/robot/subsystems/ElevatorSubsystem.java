package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public class ElevatorSubsystem {
     ShuffleboardTab elevatorTestingTab = Shuffleboard.getTab("ElevatorTestingTab");
     GenericEntry turns;

     private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless);
     // private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless);
    
     public void RunElevator(int polarity){
          System.out.println("Seen");
          double speed = ElevatorConstants.kElevatorMaxSpeed * polarity;
          if(speed != 0){
               m_leftMotor.set(speed);
               // m_rightMotor.set(speed);
          }
     }

     public ElevatorSubsystem(){
          turns = elevatorTestingTab.add("Spark2", 0).getEntry();
          turns.setDouble(m_leftMotor.get());
     }

}

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

     // private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless);

     public void RunElevator(int polarity){
          System.out.print("yeh");
          //m_leftMotor.isFollower();
          
          double speed = ElevatorConstants.kElevatorMaxSpeed * polarity;
          
               // m_leftMotor.set(speed);
               m_rightMotor.set(-speed);
          
     }
}

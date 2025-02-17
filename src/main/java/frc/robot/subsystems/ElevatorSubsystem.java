package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Velocity;

public class ElevatorSubsystem extends SubsystemBase{
     ShuffleboardTab elevatorTestingTab = Shuffleboard.getTab("ElevatorTestingTab");
     GenericEntry turns, turnRate, eleSpeed;


     private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless);
     private double encoderOffset;
     private double eleSetSpeed = ElevatorConstants.kElevatorMaxSpeed;

    
     public void RunElevator(int polarity){
          System.out.print(polarity);
          //m_leftMotor.isFollower();
          
          double speed = eleSetSpeed * polarity;
          
               m_leftMotor.set(speed);
               m_rightMotor.set(-speed);
          
     }
     public void zeroEleEncoder(){

          encoderOffset = m_rightMotor.getEncoder().getPosition();

     }

     public double getEleEncoder(){
          return m_rightMotor.getEncoder().getPosition() - encoderOffset;
     }



     public ElevatorSubsystem(){
          turns = elevatorTestingTab.add("Spark2 Poition", 0).getEntry();
          turnRate = elevatorTestingTab.add("Spark2 Velocity", 0).getEntry();
          eleSpeed = elevatorTestingTab.add("SetSpeed",eleSetSpeed).getEntry();
          zeroEleEncoder();
          turns.setDouble(getEleEncoder());
          turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
          //eleSpeed.setDouble(eleSetSpeed);
          
     }

     
     @Override
     public void periodic() {
         // TODO Auto-generated method stub
         turns.setDouble(getEleEncoder());
         turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
         eleSetSpeed = eleSpeed.get().getDouble();

         //super.periodic();
     }

}

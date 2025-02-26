package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.math.controller.SimpleMotorFeedForward;
import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.co

public class ElevatorSubsystem extends SubsystemBase{
     ShuffleboardTab elevatorTestingTab = Shuffleboard.getTab("ElevatorTestingTab");
     GenericEntry turns, turnRate, eleSpeed, eleSP;

     private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless);
     private double encoderOffset;
     private double eleSetSpeed = ElevatorConstants.kElevatorMaxSpeed;

     //private SparkMaxConfig config;

     public void zeroEleEncoder(){
          encoderOffset = m_rightMotor.getEncoder().getPosition();
     }

     public double getEleEncoder(){
          return m_rightMotor.getEncoder().getPosition() - encoderOffset;
     }
     
     public void RunElevator(double polarity){
          double speed = eleSetSpeed * polarity;
               m_leftMotor.set(speed);
     }

     public ElevatorSubsystem(){
          turns = elevatorTestingTab.add("Spark2 Poition", 0).getEntry();
          turnRate = elevatorTestingTab.add("Spark2 Velocity", 0).getEntry();
          eleSpeed = elevatorTestingTab.add("SetSpeed",eleSetSpeed).getEntry();
          eleSP = elevatorTestingTab.add("SetPoint",0).getEntry();
          zeroEleEncoder();
          turns.setDouble(getEleEncoder());
          turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());

         m_rightMotor.getClosedLoopController();
     }

     public Command setToHeight(){
           return this.runOnce(() -> m_rightMotor.getClosedLoopController().setReference(eleSP.get().getDouble()+encoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0));
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

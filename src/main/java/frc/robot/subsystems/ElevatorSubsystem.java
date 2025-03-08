package frc.robot.subsystems;
import com.ctre.phoenix6.signals.DiffPIDOutput_PIDOutputModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLimitSwitch;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.controller.SimpleMotorFeedForward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.controller.ElevatorFeedforward;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.contr



public class ElevatorSubsystem extends SubsystemBase{
     ShuffleboardTab elevatorTestingTab = Shuffleboard.getTab("ElevatorTestingTab");
     GenericEntry turns, turnRate, eleSpeed, eleSP, speedRight, speedLeft, encoderOffsetSB;


     private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless); //Master
     private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless); //Has Limit Switch
     //private DiffPIDOutput_PIDOutputModeValue
     private double encoderOffset;
     private double eleSetSpeed = ElevatorConstants.kElevatorMaxSpeed;
     double eleSetpoint;
     double maxEleSp = 0;
     double minEleSp = 0;
     
     //shuffleboard stuff
     SparkLimitSwitch topSwitch = m_rightMotor.getForwardLimitSwitch();
     SparkLimitSwitch bottomSwitch = m_rightMotor.getReverseLimitSwitch();


     private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, 
                         ElevatorConstants.kV, ElevatorConstants.kA);
     private ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,new TrapezoidProfile.Constraints(
              1000,
              100));
     //private SparkMaxConfig config;

     public void zeroEleEncoder(){
          encoderOffset = m_leftMotor.getEncoder().getPosition();
     }

     public double getEleEncoder(){
          return m_leftMotor.getEncoder().getPosition() - encoderOffset;
     }

     public double getSpeedLeft(){
          return m_leftMotor.getAppliedOutput();
     }

     public double getSpeedRight(){
          return m_rightMotor.getAppliedOutput();
     }
     
     public void RunElevator(double polarity){
          // if((!topSwitch.isPressed() && polarity > 1) && (!bottomSwitch.isPressed() && polarity < 1)){
          //idk
               double speed = polarity * 0.5;

               //double speed = eleSetSpeed * polarity;
               m_leftMotor.set(speed);;
          // }
     }

     public ElevatorSubsystem(){
          turns = elevatorTestingTab.add("Spark2 Poition", 0).getEntry();
          turnRate = elevatorTestingTab.add("Spark2 Velocity", 0).getEntry();
          eleSpeed = elevatorTestingTab.add("SetSpeed",eleSetSpeed).getEntry();
          eleSP = elevatorTestingTab.add("SetPoint",0).getEntry();
          
          //shuffle board stuff
          speedLeft = elevatorTestingTab.add("LeftSpeed", 0).getEntry();
          speedRight = elevatorTestingTab.add("Feedforward Value", 0).getEntry();
          encoderOffsetSB = elevatorTestingTab.add("Encoder Offset", 0).getEntry();


          zeroEleEncoder();
          turns.setDouble(getEleEncoder());
          turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
          elevatorPID.setTolerance(.25);
     }

     public void setToHeight(){
          // if((eleSetpoint >= maxEleSp) && (eleSetpoint <= minEleSp)){
          //      m_rightMotor.getClosedLoopController().setReference(eleSetpoint+encoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
          // }
          System.out.println("Seen");
          m_leftMotor.getClosedLoopController().setReference(eleSetpoint+encoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
     }

     @Override
     public void periodic() {
         // TODO Auto-generated method stub
         turns.setDouble(getEleEncoder());
         turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
         eleSetSpeed = eleSpeed.get().getDouble();
         
         //shuffleboard stuff
         eleSetpoint = eleSP.getDouble(eleSetpoint);
         speedLeft.setDouble(getSpeedLeft());
         encoderOffsetSB.setDouble(encoderOffset);
         speedRight.setDouble(elevatorFeedForward.calculate(0));

         if(m_leftMotor.getReverseLimitSwitch().isPressed()){
          zeroEleEncoder();
          // m_leftMotor.set(0);
          // m_rightMotor.set(0);
          System.out.println("Bottom Hit" + eleSetpoint);
          }

         if(m_leftMotor.getForwardLimitSwitch().isPressed()){
          // m_leftMotor.set(0);
          // m_rightMotor.set(0);
           System.out.println("Bottom Hit" + eleSetpoint);
          }
     }
}

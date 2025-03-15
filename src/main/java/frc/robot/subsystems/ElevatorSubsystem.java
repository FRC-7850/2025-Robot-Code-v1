package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.signals.DiffPIDOutput_PIDOutputModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
     GenericEntry turns, turnRate, eleSpeed, eleSP, speedRight, speedLeft, FeedForward, maxAcceleration, encoderOffsetSB, kV, calculatedValue;


     private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless); //Master
     private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless); //Has Limit Switch
     //private DiffPIDOutput_PIDOutputModeValue
     private double encoderOffset;
     double eleSetpoint;
     double maxEleSp = 0;
     double minEleSp = 0;
     double voltage;
     
     //shuffleboard stuff
     SparkLimitSwitch topSwitch = m_rightMotor.getForwardLimitSwitch();
     SparkLimitSwitch bottomSwitch = m_rightMotor.getReverseLimitSwitch();

     private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG,
                         ElevatorConstants.kV, ElevatorConstants.kA);
     private ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(
              130,
              130));
     //private SparkMaxConfig config;
     //24,36

     public void zeroEleEncoder(){
          encoderOffset = m_leftMotor.getEncoder().getPosition();
     }

     public double getEleEncoder(){
          return m_leftMotor.getEncoder().getPosition() - encoderOffset;
     }

     public double getSpeedLeft(){
          return m_leftMotor.getEncoder().getVelocity();
     }

     public double getSpeedRight(){
          return m_rightMotor.getAppliedOutput();
     }

     public void setAntiGravity(){
          m_leftMotor.setVoltage(Constants.ElevatorConstants.kG);
     }

     public void gotoPosition(){
          m_leftMotor.setVoltage(voltage);
          calculatedValue.setDouble(voltage);
          FeedForward.setDouble(elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity));
     }

     public void calculatePID(){
          voltage = elevatorPID.calculate(getEleEncoder())
          + elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity);
     }

     public void calculatePIDFineTune(){
          elevatorPID.setGoal(getEleEncoder());
          voltage = elevatorPID.calculate(getEleEncoder())
          + elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity);
     }

     public boolean PIDAtGoal(){
          return elevatorPID.atGoal();
     }

     
     
     public void RunElevator(double polarity){
          // if((!topSwitch.isPressed() && polarity > 1) && (!bottomSwitch.isPressed() && polarity < 1)){
          //idk
               double speed = polarity * 0.5;

               //double speed = eleSetSpeed * polarity;
               m_leftMotor.set(speed);;
               //gotoPosition(getEleEncoder());
          // }
     }

     public void setSetpoint(){
          elevatorPID.setGoal(eleSP.get().getDouble());
     }

     public void setSetpointButton(double goal){
          elevatorPID.setGoal(goal);
     }

     public ElevatorSubsystem(){
          turns = elevatorTestingTab.add("Encoder Readout", 0).getEntry();
          turnRate = elevatorTestingTab.add("Spark2 Velocity", 0).getEntry();
          eleSP = elevatorTestingTab.add("Setpoint", 0).getEntry();
          calculatedValue = elevatorTestingTab.add("Votage", 0).getEntry();
          FeedForward = elevatorTestingTab.add("FF", 0).getEntry();

          zeroEleEncoder();
          turns.setDouble(getEleEncoder());
          turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
          elevatorPID.setTolerance(.25);
     }

     @Override
     public void periodic() {
          //elevatorFeedForward.setKa(kV.get().getDouble());
          //elevatorPID.setP(kV.get().getDouble());
         // TODO Auto-generated method stub

         turns.setDouble(getEleEncoder());
         turnRate.setDouble(getSpeedLeft());

         if(m_leftMotor.getReverseLimitSwitch().isPressed()){
          zeroEleEncoder();
          }

     //     if(m_leftMotor.getForwardLimitSwitch().isPressed()){
     //      }
          }
     }

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLimitSwitch;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.ElevatorFeedforward;

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
          calculatePID();
          m_leftMotor.setVoltage(voltage);
          calculatedValue.setDouble(voltage);
          FeedForward.setDouble(elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity));
     }

     public void calculatePID(){
          voltage = elevatorPID.calculate(getEleEncoder())
          + elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity);
     }

     // public void calculatePIDFineTune(){
     //      elevatorPID.setGoal(getEleEncoder());
     //      voltage = elevatorPID.calculate(getEleEncoder())
     //      + elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity);
     // }

     public boolean AtGoal(){
          return elevatorPID.atGoal();
     }

     // public void setGoalManual(){
     //      elevatorPID.setGoal(eleSP.get().getDouble());
     // }

     
     
     public void RunElevator(double polarity){
          // if((!topSwitch.isPressed() && polarity > 1) && (!bottomSwitch.isPressed() && polarity < 1)){
          //idk
               double speed = polarity * 0.5;

               //double speed = eleSetSpeed * polarity;
               m_leftMotor.set(speed);;
          // }
     }

     // public void setSetpoint(){
     //      elevatorPID.setGoal(eleSP.get().getDouble());
     // }

     public void setSetpointButton(double goal){
          elevatorPID.setGoal(goal);
          System.out.println("Run elevator to setpoint");
          gotoPosition();
     }

     public ElevatorSubsystem(){
          elevatorPID.setGoal(0);
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
         turns.setDouble(getEleEncoder());
         turnRate.setDouble(getSpeedLeft());
         gotoPosition();

         if(m_leftMotor.getReverseLimitSwitch().isPressed()){
          zeroEleEncoder();
          }
     }
 }

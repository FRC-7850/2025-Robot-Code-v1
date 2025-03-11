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
     GenericEntry turns, turnRate, eleSpeed, eleSP, speedRight, speedLeft, instantaneousAcceleration, maxAcceleration, encoderOffsetSB, kV, calculatedValue;


     private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless); //Master
     private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless); //Has Limit Switch
     //private DiffPIDOutput_PIDOutputModeValue
     private double encoderOffset;
     private double eleSetSpeed = ElevatorConstants.kElevatorMaxSpeed;
     double eleSetpoint;
     double maxEleSp = 0;
     double minEleSp = 0;
     double derivativeMain = 0;
     double derivativeMax = 0;
     double[] deltaV = {0,0};
     
     //shuffleboard stuff
     SparkLimitSwitch topSwitch = m_rightMotor.getForwardLimitSwitch();
     SparkLimitSwitch bottomSwitch = m_rightMotor.getReverseLimitSwitch();

     private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG,
                         ElevatorConstants.kV, ElevatorConstants.kA);
     private ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(
              24,
              36));
     //private SparkMaxConfig config;

     public void zeroEleEncoder(){
          encoderOffset = m_leftMotor.getEncoder().getPosition();
     }

     public double getEleEncoder(){
          return m_leftMotor.getEncoder().getPosition() - encoderOffset;
     }

     public double getSpeedLeft(){
          return m_leftMotor.getEncoder().getVelocity();
     }

     public void getAccelerationLeft(){
         deltaV[0] = deltaV[1];
         deltaV[1] = getSpeedLeft();
         double derivative = deltaV[1] - deltaV[0];
         derivativeMain = derivative;
         if(Math.signum(derivative) > Math.signum(derivativeMax)){
          derivativeMax = Math.signum(derivative);
         }
     }

     public double getSpeedRight(){
          return m_rightMotor.getAppliedOutput();
     }

     public void setAntiGravity(){

          m_leftMotor.setVoltage(Constants.ElevatorConstants.kG);
     }

     public void gotoPosition(double goal){
          
          elevatorPID.setGoal(goal);
          m_leftMotor.setVoltage(
               elevatorPID.calculate(getEleEncoder())
                   + elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity));
     }

     public void gotoPositionPID(double goal){
          
          elevatorPID.setGoal(goal);
          m_leftMotor.setVoltage(
               elevatorPID.calculate(getEleEncoder())
                   );
     }

     public boolean PIDAtSP(){
          return elevatorPID.atGoal();
     }

     public void setFFAlone(){

          m_leftMotor.setVoltage(
               elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity));
     }

     public double getShuffleSP(){
          return eleSetpoint;
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

     public ElevatorSubsystem(){
          turns = elevatorTestingTab.add("Spark2 Poition", 0).getEntry();
          turnRate = elevatorTestingTab.add("Spark2 Velocity", 0).getEntry();
          eleSpeed = elevatorTestingTab.add("SetSpeed",eleSetSpeed).getEntry();
          eleSP = elevatorTestingTab.add("SetPoint",0).getEntry();
          instantaneousAcceleration = elevatorTestingTab.add("Instantaneous Acceleration", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
          maxAcceleration = elevatorTestingTab.add("Max Acceleration", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
          speedLeft = elevatorTestingTab.add("Speed/Velocity", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
          speedRight = elevatorTestingTab.add("Feedforward Value", 0).getEntry();
          encoderOffsetSB = elevatorTestingTab.add("Encoder Offset", 0).getEntry();
          kV = elevatorTestingTab.add("KG", 0).getEntry();
          calculatedValue = elevatorTestingTab.add("Calculated Velocity", 0).getEntry();
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
         eleSetSpeed = eleSpeed.get().getDouble();
         eleSetpoint = eleSP.getDouble(eleSetpoint);
         speedLeft.setDouble(getSpeedLeft());
         encoderOffsetSB.setDouble(encoderOffset);
         speedRight.setDouble(elevatorFeedForward.calculate(0));
         calculatedValue.setDouble(
          elevatorPID.calculate(getEleEncoder())
                   + elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity));

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

         /*  elevatorPID.setGoal(eleSetpoint);

          m_leftMotor.setVoltage(
               elevatorPID.calculate(getEleEncoder())
                   + elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity));
         */}
////////////////////////testing elevator setpoints as a command/////////////////////////////////////
         public Command ElevatorPIDCommand(double elevatorSetpoint) {
          
          return 
                  // Run the shooter flywheel at the desired setpoint using feedforward and feedback
                  run(
                      () -> {
                         gotoPosition(eleSetpoint);
                        }
      
                  // Wait until the shooter has reached the setpoint, and then run the feeder
                  );//.until(elevatorPID::atSetpoint);
              
        }
////////////////////////////////////////////////////////////
     }

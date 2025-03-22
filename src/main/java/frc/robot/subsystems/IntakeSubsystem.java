package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//WPILib
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Rev
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

     //File Structure
//Constants  
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
     //Shuffleboard Entries
     ShuffleboardTab intakeTestingTab = Shuffleboard.getTab("IntakeTestingTab");
     GenericEntry encoderReadout, armSP, calculatedVoltage;

     //Definitions
     //Set followers and PID constants in Rev client, because following only right motors are used in code
     private final SparkMax armMotorLeft = new SparkMax(CanIDConstants.kArmLeftCanId, MotorType.kBrushed);
     // private final SparkMax armMotorRight = new SparkMax(CanIDConstants.kArmRightCanId, MotorType.kBrushed); //Unused
     private final SparkMax intakeMotorLeft = new SparkMax(CanIDConstants.kIntakeLeftCanId, MotorType.kBrushless);
     private final SparkMax intakeMotorRight = new SparkMax(CanIDConstants.kIntakeRightCanId, MotorType.kBrushless);
     public double armEncoderOffset;
     public double setpointReference;
     private ArmFeedforward armFeedForward = new ArmFeedforward(
          IntakeConstants.kS, 
          IntakeConstants.kG,
          IntakeConstants.kV, 
          IntakeConstants.kA);
     private ProfiledPIDController armPID = new ProfiledPIDController(
          IntakeConstants.kP, 
          IntakeConstants.kI, 
          IntakeConstants.kD, 
          new TrapezoidProfile.Constraints(
              40,
              40));    
     //Subsystem Method
     public IntakeSubsystem(){
          //Shuffleboard Entry Creation
          encoderReadout = intakeTestingTab.add("Encoder Readout", 0).getEntry();
          encoderReadout = intakeTestingTab.add("Setpoint", 0).getEntry();
          encoderReadout = intakeTestingTab.add("Calculated Voltage", 0).getEntry();

          //PID nullified on startup
          SetGoal(getArmEncoder());
     }

          //Reference Methods
     //Arm

     public double getArmEncoder(){
          return armMotorLeft.getAbsoluteEncoder().getPosition() - armEncoderOffset;
     }

     public void SetGoal(double goal){
         armPID.setGoal(goal);
         setpointReference = goal;
     }

     public double calculatePID(){
          return armPID.calculate(getArmEncoder()) + 
          armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity);
     }

     public void gotoPosition(){
          double voltage = calculatePID();
          armMotorLeft.setVoltage(voltage);
          calculatedVoltage.setDouble(voltage);
     }

     public boolean AtGoal(){
          if(getArmEncoder() > (setpointReference + IntakeConstants.kIntakePIDCutoffRange) || getArmEncoder() < (setpointReference - IntakeConstants.kIntakePIDCutoffRange)){
               return true;
          }
          return false;
     }


     //Intake
     public void Shoot(){       
          intakeMotorRight.set(-1);
          intakeMotorLeft.set(1);
     }

     public void Intake(){       
          intakeMotorRight.set(1);
          intakeMotorLeft.set(-1);
     }

     public void GetIntakeCurrent(){       
          
     }

     @Override
     public void periodic() {
          //Shuffleboard Update
          encoderReadout.setDouble(getArmEncoder());

          //Run PID
          gotoPosition();
     }
}
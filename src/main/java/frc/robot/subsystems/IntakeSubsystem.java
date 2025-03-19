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

          //Initialize
          // ZeroArmEncoder();
     }

          //Reference Methods
     //Arm
     public void ArmFineTune(double speed){     
          armMotorLeft.set(speed * IntakeConstants.kArmFineTuneSpeed);
     }

     public double getArmEncoder(){
          return armMotorLeft.getAbsoluteEncoder().getPosition();
     }

          // Unused
     // public void zeroArmEncoder(){
     //      armEncoderOffset = armMotorLeft.getEncoder().getPosition();
     // }

     public void SetGoal(double goal){
         armPID.setGoal(goal);
     }

     public double CalculateKG(){
          //Uses 4 peicewise equations depending on gravity on the arm (should really be a function)
          if(getArmEncoder() < -99 || (getArmEncoder() > -99 && getArmEncoder() < -77)){
               return .33;
          }
          if(getArmEncoder() > -77 && getArmEncoder() < -57){
               return .36;
          }
          if(getArmEncoder() > -57 && getArmEncoder() < -37){
               return .33;
          }
          if(getArmEncoder() > -37 && getArmEncoder() < -17){
               return .28;
          }
          if(getArmEncoder() > 0 || (getArmEncoder() > -17 && getArmEncoder() < 0)){
               return .15;          
          }
          //Fallback
          return(armFeedForward.getKg());
     }

     public void setAntiGravity(){
          armMotorLeft.setVoltage(CalculateKG());
     }

     public double calculatePID(){
          return armPID.calculate(getArmEncoder()) + 
          armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity) 
          + (CalculateKG()); //Initially multiplied by 0.75 when copied into branch
     }

     public void gotoPosition(){
          double voltage = calculatePID();
          armMotorLeft.setVoltage(voltage);
          calculatedVoltage.setDouble(voltage);
     }

     public boolean AtGoal(double setpoint){
          if(getArmEncoder() > (setpoint + 5) || getArmEncoder() < (setpoint + 5)){
               return true;
          }
          return false;
     }


     //Intake
     public void Shoot(){       

     }

     public void Intake(){       

     }

     public void GetIntakeCurrent(){       
          
     }

     @Override
     public void periodic() {
          //Shuffleboard Update
          encoderReadout.setDouble(getArmEncoder());
     }
}
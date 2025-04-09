package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase{
     ShuffleboardTab intakeTestingTab = Shuffleboard.getTab("IntakeTestingTab");
     GenericEntry encoderReadout;
     GenericEntry kP, kV, SetPoint, instantVelocity, Eq1, kA, kD, kG, kI, setVoltage;
     public double setpointReference;

          private ArmFeedforward armFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG,
          ArmConstants.kV, ArmConstants.kA);
     private ProfiledPIDController armPID = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new TrapezoidProfile.Constraints(
              20,
              10));

     private final SparkMax m_armMotorLeft = new SparkMax(OIConstants.kArmCanIDLeft, MotorType.kBrushless);
     // private final SparkMax m_armMotorRight = new SparkMax(OIConstants.kArmCanIDRight, MotorType.kBrushed); //Unused
     private final SparkMax m_intakeMotorLeft = new SparkMax(OIConstants.kIntakeCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_intakeMotorRight = new SparkMax(OIConstants.kIntakeCanIDRight, MotorType.kBrushless);
     private double encoderOffset = 2.061;
     public double voltage;
     public double condom;
     //For precise shots in the barge where only the top motors push it in. true when shooting backward, false when shooting forward
     public boolean bargeFlip = true;
     
     public void RunArm(double polarity){
               m_armMotorLeft.set(polarity);
     }

     public void RunIntake(double polarity){          
        m_intakeMotorRight.set(polarity);
        m_intakeMotorLeft.set(-polarity);
        System.out.println("RunIntake" + polarity);
   }

     public void RunIntakePrecise(int status){
     //   if(bargeFlip){
     //      m_intakeMotorRight.set(-status);
     //   }
     //   else{m_intakeMotorLeft.set(status);}
     m_intakeMotorRight.set(-status * 0.65);
     m_intakeMotorLeft.set(status * 0.65);
     }

     public void FlipBarge(boolean flip){
          bargeFlip = flip;
     }

     public double getArmEncoder(){
          return m_armMotorLeft.getAlternateEncoder().getPosition() * 6.2832 + encoderOffset;
     }

     public void zeroArmEncoder(){
         encoderOffset = m_armMotorLeft.getEncoder().getPosition();
     }

     //setpoints are commented out nwo 🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴
     public void gotoPosition(){
          calculatePID();
          m_armMotorLeft.setVoltage(voltage);
          instantVelocity.setDouble(voltage);
     }

     public void calculatePID(){
          voltage = armPID.calculate(getArmEncoder()) + armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity);
     }

     public void RunPIDOnStartup(){
          condom = armPID.calculate(getArmEncoder()) + armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity);
     }

     // public void setAntiGravity(){
     //      m_armMotorLeft.setVoltage(armFeedForward.getKg());
     // }

     public void setAntiGravityOff(){
          m_armMotorLeft.set(0);
     }

     // public double CalculateKG(){
     //      //Uses 4 peicewise equations depending on gravity on the arm
     //      if(getArmEncoder() < -99 || (getArmEncoder() > -99 && getArmEncoder() < -77)){
     //           return .33;
     //      }
     //      if(getArmEncoder() > -77 && getArmEncoder() < -57){
     //           return .36;
     //      }
     //      if(getArmEncoder() > -57 && getArmEncoder() < -37){
     //           return .33;
     //      }
     //      if(getArmEncoder() > -37 && getArmEncoder() < -17){
     //           return .28;
     //      }
     //      if(getArmEncoder() > 0 || (getArmEncoder() > -17 && getArmEncoder() < 0)){
     //           return .15;          
     //      }
     //      return(armFeedForward.getKg());
     // }

     // public void setSetpoint(){
     //      armPID.setGoal(SetPoint.get().getDouble());
     // }

     public void setSetpointButton(double goal){
          armPID.setGoal(goal);
     }

     public boolean PIDAtGoal(){
          return armPID.atGoal();
     }

     // public void setSetpointneg80(){
     //      armPID.setGoal(-96);
     // }

      public boolean AtGoal(){
          return armPID.atGoal();
     }

     // public void setGoalManual(){
     //      armPID.setGoal(SetPoint.get().getDouble());
     // }

     public boolean atSafeZone(){
          return getArmEncoder()<Constants.IntakeConstants.kIntakeSafeEncoderValue;
     }

     public IntakeSubsystem(){
          kA = intakeTestingTab.add("Ka", 0).getEntry(); //100
          kD = intakeTestingTab.add("Kd", 0).getEntry();
          kP = intakeTestingTab.add("Kp", 0).getEntry();
          SetPoint = intakeTestingTab.add("Setpoint", 0).getEntry();
          encoderReadout = intakeTestingTab.add("Encoder Readout", 0).getEntry();
          instantVelocity = intakeTestingTab.add("Voltage", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
          Eq1 = intakeTestingTab.add("Equation 1 Constant", 4.3617).getEntry();
          kV = intakeTestingTab.add("kV", 0).getEntry(); //20
          kG = intakeTestingTab.add("KG", 0).getEntry();
          kI = intakeTestingTab.add("KI", 0).getEntry();
          armPID.setTolerance(.1);
          setVoltage = intakeTestingTab.add("Input Test Voltage", 0).getEntry();
          armPID.setGoal(getArmEncoder());
          RunPIDOnStartup();
     }

     @Override
     public void periodic() {
          gotoPosition();
          encoderReadout.setDouble(getArmEncoder());
     }
}


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
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase{
     ShuffleboardTab intakeTestingTab = Shuffleboard.getTab("IntakeTestingTab");
     GenericEntry encoderReadout;
     GenericEntry kP, kV, SetPoint, instantVelocity, Eq1, kA, kD, kG, kI;

          private ArmFeedforward armFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG,
          ArmConstants.kV, ArmConstants.kA);
     private ProfiledPIDController armPID = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new TrapezoidProfile.Constraints(
              40,
              40));

     private final SparkMax m_armMotorLeft = new SparkMax(OIConstants.kArmCanIDLeft, MotorType.kBrushless);
     // private final SparkMax m_armMotorRight = new SparkMax(OIConstants.kArmCanIDRight, MotorType.kBrushed); //Unused
     private final SparkMax m_intakeMotorLeft = new SparkMax(OIConstants.kIntakeCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_intakeMotorRight = new SparkMax(OIConstants.kIntakeCanIDRight, MotorType.kBrushless);
     private double encoderOffset;
     public double voltage;

     public void RunArm(double polarity){
               m_armMotorLeft.set(polarity);
     }

     public void RunIntake(double polarity){          
        m_intakeMotorRight.set(polarity);
        m_intakeMotorLeft.set(-polarity);
        System.out.println("RunIntake" + polarity);
   }

     public double getArmEncoder(){
          return m_armMotorLeft.getEncoder().getPosition() - encoderOffset;
     }

     public double getArmAbsoluteEncoder(){
          return m_armMotorLeft.getAbsoluteEncoder().getPosition();
     }

     public void zeroArmEncoder(){
         encoderOffset = m_armMotorLeft.getEncoder().getPosition();
     }

     //setpoints are commented out nwo 🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴
     public void gotoPosition(){
          // armPID.setGoal(SetPoint.get().getDouble());
          // armPID.setI(kD.get().getDouble());
          // armPID.setD(kI.get().getDouble());
          calculatePID();
          m_armMotorLeft.setVoltage(voltage);
          instantVelocity.setDouble(voltage);
     }

     public void calculatePID(){
          voltage =  armPID.calculate(getArmEncoder()) + armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity) + (CalculateKG() * 0.75);
     }

     public void calculatePIDFineTune(){
          armPID.setGoal(getArmEncoder());
          voltage =  armPID.calculate(getArmEncoder()) + armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity) + (CalculateKG() * 0.75);
     }

     public void calculatePIDwithSetpoint(){
          voltage =  armPID.calculate(getArmEncoder()) + armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity) + (CalculateKG() * 0.75);
     }
     public void setAntiGravity(){
          m_armMotorLeft.setVoltage(CalculateKG());
     }
     public void setAntiGravityOff(){
          m_armMotorLeft.set(0);
     }

     public double CalculateKG(){
          //Uses 4 peicewise equations depending on gravity on the arm
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
          return(armFeedForward.getKg());
     }

     public void setSetpoint(){
          armPID.setGoal(SetPoint.get().getDouble());
     }

     public void setSetpointButton(double goal){
          armPID.setGoal(goal);
     }

     public boolean PIDAtGoal(){
          return armPID.atGoal();
     }

     public void setSetpointneg80(){
          armPID.setGoal(-96);
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
          armPID.setTolerance(1);
     }

     @Override
     public void periodic() {
          encoderReadout.setDouble(getArmEncoder());
          // armPID.setD(kD.get().getDouble());
          // armPID.setP(kP.get().getDouble());
          // armFeedForward.setKv(kV.get().getDouble());
          // armFeedForward.setKa(kA.get().getDouble());
     }
}


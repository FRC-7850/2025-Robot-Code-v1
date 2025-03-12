package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;
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
     GenericEntry kV, SetPoint, instantVelocity;

          private ArmFeedforward armFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG,
          ArmConstants.kV, ArmConstants.kA);
     private ProfiledPIDController armPID = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new TrapezoidProfile.Constraints(
              24,
              36));

    //Set followers and PID constants in Rev client, because following only right motors are used in code
     private final SparkMax m_armMotorLeft = new SparkMax(OIConstants.kArmCanIDLeft, MotorType.kBrushed);
     private final SparkMax m_armMotorRight = new SparkMax(OIConstants.kArmCanIDRight, MotorType.kBrushed);
     private final SparkMax m_intakeMotorLeft = new SparkMax(OIConstants.kIntakeCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_intakeMotorRight = new SparkMax(OIConstants.kIntakeCanIDRight, MotorType.kBrushless);
    
     public void RunArm(double polarity){
               m_armMotorLeft.set(polarity);
               m_armMotorRight.set(polarity);

     }

     public void RunIntake(double polarity){          
        m_intakeMotorRight.set(polarity);
        m_intakeMotorLeft.set(-polarity);
        System.out.println("RunIntake" + polarity);
   }

     public double getArmEncoder(){
          return m_armMotorLeft.getEncoder().getPosition();
     }

     public void gotoPosition(){
          armPID.setGoal(SetPoint.get().getDouble());
          double voltage =  armPID.calculate(getArmEncoder()) + armFeedForward.calculate(armPID.getSetpoint().position,armPID.getSetpoint().velocity) + CalculateKG();
          m_armMotorLeft.setVoltage(voltage);
          m_armMotorRight.setVoltage(voltage);
          System.out.println("gotoPosition" + voltage);
          instantVelocity.setDouble(voltage);
     }

     public void setAntiGravity(){
          m_armMotorLeft.setVoltage(ArmConstants.kG);
     }
     public void setAntiGravityOff(){
          m_armMotorLeft.set(0);
     }

     public double CalculateKG(){
          //Uses 4 peicewise equations depending on gravity on the arm
          if(getArmEncoder() <= .136){
              return (((1.75/0.376) * getArmEncoder()) + 4.3617);
          }else if(getArmEncoder() <= .185){
               return (((.45/0.049) * getArmEncoder()) + 3.30102);
          }else if(getArmEncoder() <= .458){
               return (((6/.273) * -getArmEncoder()) + 10.06593);
          }else if(getArmEncoder() <= .58){
               return (((5.5/.122) * -getArmEncoder()) + 23.4);
          }
          
          //Maximum kG value cap
          if(armFeedForward.getKg() > 6){
               return (6);
          }
          return(armFeedForward.getKg());
     }

     public IntakeSubsystem(){
          kV = intakeTestingTab.add("Kp", 0).getEntry();
          SetPoint = intakeTestingTab.add("Setpoint", 0).getEntry();
          encoderReadout = intakeTestingTab.add("Encoder Readout", 0).getEntry();
          instantVelocity = intakeTestingTab.add("Voltage", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
     }

     @Override
     public void periodic() {
          armPID.setP(kV.get().getDouble());
          encoderReadout.setDouble(getArmEncoder());
     }
}


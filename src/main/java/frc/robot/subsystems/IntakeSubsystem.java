package frc.robot.subsystems;
import java.time.Instant;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
     ShuffleboardTab intakeTestingTab = Shuffleboard.getTab("IntakeTestingTab");
     GenericEntry encoderReadout;
     GenericEntry adjustableArmSpeed;
     double armSpeed;

    //Set followers and PID constants in Rev client, because following only right motors are used in code
     private final SparkMax m_armMotorLeft = new SparkMax(OIConstants.kArmCanIDLeft, MotorType.kBrushed);
     private final SparkMax m_armMotorRight = new SparkMax(OIConstants.kArmCanIDRight, MotorType.kBrushed);
     private final SparkMax m_intakeMotorLeft = new SparkMax(OIConstants.kIntakeCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_intakeMotorRight = new SparkMax(OIConstants.kIntakeCanIDRight, MotorType.kBrushless);

     private double armEncoderOffset;
     private double intakeMaxSpeed = IntakeConstants.kIntakeSpeed;
     private double armMaxSpeed = IntakeConstants.kArmFineTuneSpeed;
    
     public void RunArm(double polarity){          
          double speed = armMaxSpeed * polarity;
          if (speed > 0){
               m_armMotorLeft.set(polarity);
          }else{
               m_armMotorLeft.set(polarity);
          }
     }

     public void RunIntake(double polarity){          
        double speed = intakeMaxSpeed * polarity;
        m_intakeMotorRight.set(polarity);
        m_intakeMotorLeft.set(-polarity);
   }

   /*public void RunInta(double polarity){          
     new InstantCommand(() -> m_intakeMotorLeft.set(polarity))
     .alongWith(new InstantCommand(() -> m_intakeMotorRight.set(-polarity)))
     .withDeadline(null)
     }*/
     
     public void zeroArmEncoder(){
          armEncoderOffset = m_armMotorRight.getEncoder().getPosition();
     }

     public double getArmEncoder(){
          return m_armMotorLeft.getEncoder().getPosition();
     }

     public void ArmToSetpoint(double setpoint){
          m_armMotorRight.getClosedLoopController().setReference(setpoint + armEncoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
     }

     public IntakeSubsystem(){
          zeroArmEncoder();
          encoderReadout = intakeTestingTab.add("Encoder Readout", 0).getEntry();
          adjustableArmSpeed = intakeTestingTab.add("Arm Speed Coefficient", 1).getEntry();
     }

     @Override
     public void periodic() {
          encoderReadout.setDouble(getArmEncoder());
          armSpeed = adjustableArmSpeed.getDouble(armSpeed);
     }
}

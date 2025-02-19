package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
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
          m_armMotorRight.set(speed);
     }

     public void RunIntake(double polarity){          
        double speed = intakeMaxSpeed * polarity;
        m_intakeMotorRight.set(speed);
        m_intakeMotorLeft.set(-speed);
   }
     
     public void zeroArmEncoder(){
          armEncoderOffset = m_armMotorRight.getEncoder().getPosition();
     }
 
     public IntakeSubsystem(){
          zeroArmEncoder();
     }

     public void ArmToSetpoint(double setpoint){
          m_armMotorRight.getClosedLoopController().setReference(setpoint + armEncoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
     }

     @Override
     public void periodic() {
     }
}

package frc.robot.subsystems;

//WPILib
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Rev
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

     //File Structure
//Constants  
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
     //Shuffleboard Entries
     ShuffleboardTab intakeTestingTab = Shuffleboard.getTab("IntakeTestingTab");
     GenericEntry encoderReadout;

     //Definitions
     //Set followers and PID constants in Rev client, because following only right motors are used in code
     private final SparkMax m_armMotorLeft = new SparkMax(CanIDConstants.kArmLeftCanId, MotorType.kBrushed);
     private final SparkMax m_armMotorRight = new SparkMax(CanIDConstants.kArmRightCanId, MotorType.kBrushed);
     private final SparkMax m_intakeMotorLeft = new SparkMax(CanIDConstants.kIntakeLeftCanId, MotorType.kBrushless);
     private final SparkMax m_intakeMotorRight = new SparkMax(CanIDConstants.kIntakeRightCanId, MotorType.kBrushless);
     private double intakeMaxSpeed = IntakeConstants.kIntakeSpeed;
     private double armMaxSpeed = IntakeConstants.kArmFineTuneSpeed;
     private double armEncoderOffset;
     double ArmSetpoint;     
    
     //Subsystem Method
     public IntakeSubsystem(){
          zeroArmEncoder();
          encoderReadout = intakeTestingTab.add("Encoder Readout", 0).getEntry();
     }

     //Reference Methods
     public void ArmFineTune(double polarity){     
     
     }

     //TODO: Implement Module
     public void RunIntake(double polarity){       

     }
     
     public void zeroArmEncoder(){
          armEncoderOffset = m_armMotorRight.getEncoder().getPosition();
     }

     public double getArmEncoder(){
          return m_armMotorRight.getEncoder().getPosition();
     }

     public void ArmToSetpoint(double setpoint){
          ArmSetpoint = setpoint;
     }

     @Override
     public void periodic() {
          //Shuffleboard Update
          encoderReadout.setDouble(getArmEncoder());

          //Arm PID
          m_armMotorRight.getClosedLoopController().setReference(ArmSetpoint + armEncoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
     }
}

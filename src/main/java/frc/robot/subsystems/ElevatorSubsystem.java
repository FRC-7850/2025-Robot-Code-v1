package frc.robot.subsystems;
import com.ctre.phoenix6.signals.DiffPIDOutput_PIDOutputModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
     GenericEntry turns, turnRate, eleSpeed, eleSP;


     private final SparkMax m_leftMotor = new SparkMax(OIConstants.kElevatorCanIDLeft, MotorType.kBrushless);
     private final SparkMax m_rightMotor = new SparkMax(OIConstants.kElevatorCanIDRight, MotorType.kBrushless);

     //private DiffPIDOutput_PIDOutputModeValue
     private double encoderOffset;
     private double eleSetSpeed = ElevatorConstants.kElevatorMaxSpeed;

     private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, 
                         ElevatorConstants.kV, ElevatorConstants.kA);
     private ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,new TrapezoidProfile.Constraints(
              1000,
              100));
     //private SparkMaxConfig config;
    
     public void RunElevator(int polarity){
          System.out.print(polarity);
          //m_leftMotor.isFollower();
        
          
          double speed = eleSetSpeed * polarity;
          
               //m_leftMotor.set(-speed);
               m_rightMotor.set(speed);

               
               
               
          
     }
     public void zeroEleEncoder(){

          encoderOffset = m_rightMotor.getEncoder().getPosition();

     }

     public double getEleEncoder(){
          
          return m_rightMotor.getEncoder().getPosition() - encoderOffset;

     }
 



     public ElevatorSubsystem(){
          turns = elevatorTestingTab.add("Spark2 Poition", 0).getEntry();
          turnRate = elevatorTestingTab.add("Spark2 Velocity", 0).getEntry();
          eleSpeed = elevatorTestingTab.add("SetSpeed",eleSetSpeed).getEntry();
          eleSP = elevatorTestingTab.add("SetPoint",0).getEntry();
          zeroEleEncoder();
          turns.setDouble(getEleEncoder());
          turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
          elevatorPID.setTolerance(.25);
          //config.inverted(true);
          //config.idleMode(SparkBaseConfig.IdleMode.kBrake);
          
          //m_rightMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
          
     }



     public void setToHeight(double setpoint){
          m_rightMotor.getClosedLoopController().setReference(eleSP.get().getDouble()+encoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
     }

     @Override
     public void periodic() {
         // TODO Auto-generated method stub
         turns.setDouble(getEleEncoder());
         turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
         eleSetSpeed = eleSpeed.get().getDouble();

         //super.periodic();
     }

}

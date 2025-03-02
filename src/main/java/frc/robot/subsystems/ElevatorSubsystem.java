package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//WPILib
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.ElevatorFeedforward;

//Rev
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

     //File Structure
//Constants
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase{
     //Shuffleboard Entries
     ShuffleboardTab elevatorDebug = Shuffleboard.getTab("Elevator Debug");
     GenericEntry turns, turnRate, eleSpeed, eleSP, kP, kI, kD;

     //Definitions
     private final SparkMax m_rightMotor = new SparkMax(CanIDConstants.kElevatorRightCanId, MotorType.kBrushless);
     SparkClosedLoopController elevatorController = m_rightMotor.getClosedLoopController();
     SparkFlexConfig config = new SparkFlexConfig();
     SparkLimitSwitch topSwitch = m_rightMotor.getForwardLimitSwitch();
     SparkLimitSwitch bottomSwitch = m_rightMotor.getReverseLimitSwitch();
     private double encoderOffset;
     private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
          ElevatorConstants.kS, 
          ElevatorConstants.kG, 
          ElevatorConstants.kV, 
          ElevatorConstants.kA
     );
     double elevatorSetpoint;
     double maxEleSP = ElevatorConstants.kMaxEleSP;
     double minEleSP = ElevatorConstants.kMinEleSP;


     //Subsystem Method
     public ElevatorSubsystem(){
          //Shuffleboard Entry Creation
          turns = elevatorDebug.add("Spark Poition", 0).getEntry();
          turnRate = elevatorDebug.add("Spark Velocity", 0).getEntry();
          eleSP = elevatorDebug.add("SetPoint",0).getEntry();
          kP = elevatorDebug.add("kP", 0.5).getEntry();
          kI = elevatorDebug.add("kP", 0.5).getEntry();
          kD = elevatorDebug.add("kP", 0.5).getEntry();

          //Initialize
          turns.setDouble(getEleEncoder());
          turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());
          zeroEleEncoder();  
     }

     //Reference Methods
     public void zeroEleEncoder(){
          encoderOffset = m_rightMotor.getEncoder().getPosition();
     }

     public double getEleEncoder(){
          return m_rightMotor.getEncoder().getPosition() - encoderOffset;
     }
     
     public void ElevatorFineTune(double input){
          if((elevatorSetpoint >= maxEleSP) && (elevatorSetpoint <= minEleSP)){
               elevatorSetpoint = elevatorSetpoint + (input * ElevatorConstants.kArmFineTuneSpeed);
          }
     }

     public void setToHeight(double setpoint){
          elevatorSetpoint = setpoint;
     }

     @Override
     public void periodic() {
         //Shuffleboard Update
         turns.setDouble(getEleEncoder());
         turnRate.setDouble(m_rightMotor.getEncoder().getVelocity());

         //Debug PID value set through Shuffleboard
         config.closedLoop
         .p(kP.getDouble(0.5))
         .i(kI.getDouble(0))
         .d(kD.getDouble(0));   

         //Safeguard
         if(bottomSwitch.isPressed()){zeroEleEncoder();}

         //Run PID
         elevatorController.setReference(elevatorSetpoint+encoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, elevatorFeedForward.calculate(0));
     }
}

package frc.robot.subsystems;

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
     ShuffleboardTab elevatorTestingTab = Shuffleboard.getTab("ElevatorTestingTab");
     GenericEntry turns, turnRate, eleSpeed, eleSP;

     //Definitions
     private final SparkMax m_leftMotor = new SparkMax(CanIDConstants.kElevatorLeftCanId, MotorType.kBrushless);
     private final SparkMax m_rightMotor = new SparkMax(CanIDConstants.kElevatorRightCanId, MotorType.kBrushless);
     SparkClosedLoopController elevatorController = m_rightMotor.getClosedLoopController();
     SparkFlexConfig config = new SparkFlexConfig();
     SparkLimitSwitch topSwitch = m_rightMotor.getForwardLimitSwitch();
     SparkLimitSwitch bottomSwitch = m_rightMotor.getReverseLimitSwitch();
     private double encoderOffset;
     private double eleSetSpeed = ElevatorConstants.kElevatorMaxSpeed;
     private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
          ElevatorConstants.kS, 
          ElevatorConstants.kG, 
          ElevatorConstants.kV, 
          ElevatorConstants.kA
     );
     double elevatorSetpoint;

     //Subsystem Method
     public ElevatorSubsystem(){
          //Shuffleboard Entry Creation
          turns = elevatorTestingTab.add("Spark2 Poition", 0).getEntry();
          turnRate = elevatorTestingTab.add("Spark2 Velocity", 0).getEntry();
          eleSpeed = elevatorTestingTab.add("SetSpeed",eleSetSpeed).getEntry();
          eleSP = elevatorTestingTab.add("SetPoint",0).getEntry();
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
          if((!topSwitch.isPressed() && input > 1) && (!bottomSwitch.isPressed() && input < 1)){
               elevatorSetpoint = elevatorSetpoint + input;
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
         eleSetSpeed = eleSpeed.get().getDouble();

         //Debug PID value set through Shuffleboard
         config.closedLoop
         .p(3)
         .i(3)
         .d(3);   

         //Run PID
         elevatorController.setReference(elevatorSetpoint+encoderOffset, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, elevatorFeedForward.calculate(0));
     }
}

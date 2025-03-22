package frc.robot.subsystems;

//WPILib
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//Rev
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

          //File Structure
//Constants
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase{
     //Shuffleboard Entries
     ShuffleboardTab elevatorDebug = Shuffleboard.getTab("Elevator Debug");
     GenericEntry turns, turnRate, eleSP;

     //Definitions
     private final SparkMax leftMotor = new SparkMax(CanIDConstants.kElevatorLeftCanId, MotorType.kBrushless);
     // private final SparkMax rightMotor = new SparkMax(CanIDConstants.kElevatorRightCanId, MotorType.kBrushless); //Unused
     SparkLimitSwitch topSwitch = leftMotor.getForwardLimitSwitch();
     SparkLimitSwitch bottomSwitch = leftMotor.getReverseLimitSwitch();
     private double encoderOffset;
     private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
          ElevatorConstants.kS, 
          ElevatorConstants.kG, 
          ElevatorConstants.kV, 
          ElevatorConstants.kA
     );
     private ProfiledPIDController elevatorPID = new ProfiledPIDController(
          ElevatorConstants.kP, 
          ElevatorConstants.kI, 
          ElevatorConstants.kD, 
          new TrapezoidProfile.Constraints(
              130,
              130));
     double elevatorSetpoint;
     double maxEleSP = ElevatorConstants.kMaxEleSP;
     double minEleSP = ElevatorConstants.kMinEleSP;

     //Subsystem Method
     public ElevatorSubsystem(){
          //Shuffleboard Entry Creation
          turns = elevatorDebug.add("Spark Poition", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
          turnRate = elevatorDebug.add("Spark Velocity", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
          eleSP = elevatorDebug.add("SetPoint",0).getEntry();

           //PID nullified on startup
           SetGoal(getEleEncoder());  
     }

     //Reference Methods
     public void zeroEleEncoder(){
          encoderOffset = leftMotor.getEncoder().getPosition();
     }

     public double getEleEncoder(){
          return leftMotor.getEncoder().getPosition() - encoderOffset;
     }
     
     public void ElevatorFineTune(double input){
          if((elevatorSetpoint >= maxEleSP) && (elevatorSetpoint <= minEleSP)){
               elevatorSetpoint = elevatorSetpoint + (input * ElevatorConstants.kArmFineTuneSpeed);
          }
     }

     public void SetGoal(double goal){
          elevatorPID.setGoal(goal);
          eleSP.setDouble(goal);
     }

     public double calculatePID(){
          return elevatorPID.calculate(getEleEncoder()) + 
          elevatorFeedForward.calculate(elevatorPID.getSetpoint().velocity);
     }

     public void gotoPosition(){
          leftMotor.setVoltage(calculatePID());
     }

     public boolean AtGoal(){
          return elevatorPID.atGoal();
     }
     
     @Override
     public void periodic() {
         //Shuffleboard Update
         turns.setDouble(getEleEncoder());
         turnRate.setDouble(leftMotor.getEncoder().getVelocity());      

         //Safeguard
         if(bottomSwitch.isPressed()){zeroEleEncoder();}

         //Run PID
         gotoPosition();
     }
}

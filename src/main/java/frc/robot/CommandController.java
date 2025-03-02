 /*
 *This file will hold all of the command methods and also orchestrate commands to run
 *after auto-pathing is completed.
 */

 package frc.robot;

//WPILib
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
 import edu.wpi.first.wpilibj2.command.InstantCommand;
 import edu.wpi.first.wpilibj2.command.RunCommand;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

    //File structure
 //Subsystems
 import frc.robot.subsystems.ElevatorSubsystem;
 import frc.robot.subsystems.PoseSubsystem;
 import frc.robot.subsystems.IntakeSubsystem;
 import frc.robot.subsystems.ClimbSubsystem;
 //Constants
 import frc.robot.Constants.NeoMotorConstants;
 import frc.robot.Constants.SetPointConstants;

 public class CommandController extends SubsystemBase{
    //Definitions
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    PoseSubsystem poseSubsystem = new PoseSubsystem();
    ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    int setpointSelected;
    double[] eleSetpoints = {
        SetPointConstants.kElevatorBargeSetpoint,
        SetPointConstants.kElevatorL3Setpoint,
        SetPointConstants.kElevatorL2Setpoint,
        SetPointConstants.kElevatorAlgaeOnCoralSetpoint,
        SetPointConstants.kElevatorProcessorSetpoint,
    };
    double[] armSetpoints = {
        SetPointConstants.kArmBargeSetpoint,
        SetPointConstants.kArmL3Setpoint,
        SetPointConstants.kArmL2Setpoint,
        SetPointConstants.kArmAlgaeOnCoralSetpoint,
        SetPointConstants.kArmProcessorSetpoint,
    };

    //Subsystem Method
    public CommandController(){
    }

    //Reference Methods
    public Command Path(){

    }

    public Command CancelPath(){

    }

    public Command PIDSuperCommand(){
        return new InstantCommand(() -> elevatorSubsystem.setToHeight(eleSetpoints[setpointSelected])).alongWith(IntakeSubsystem.setToHeight(eleSetpoints[setpointSelected]));
    }

    public Command ElevatorFineTune(double input){
        return new RunCommand(() -> elevatorSubsystem.ElevatorFineTune(input),elevatorSubsystem);
    }

    public Command ArmFineTune(double input){
        return new RunCommand(() -> intakeSubsystem.ArmFineTune(input),elevatorSubsystem);    }

    public Command Intake(boolean toggle){
        return this.runOnce(() -> IntakeSubsystem.Intake(toggle));
    }
    
    public Command Shoot(booelan toggle){
        return this.runOnce(() -> IntakeSubsystem.Shoot(toggle));
    }

    public Command Climb(double polarity){
        return new InstantCommand(() -> ClimbSubsystem.Climb(NeoMotorConstants.kClimberSpeed * polarity));
    }

    public Command StopClimb(){
        return new InstantCommand(() -> ClimbSubsystem.StopClimb());
    }

    //Old idea for setpoint selection
    // public void BrowseSetpointList(int i){
    //     setpointSelected = (setpointSelected == 4) ? 0 : setpointSelected + i;
    // }

    //Coral Integration?
    // public Command CoralIntake(){
    // }
 }
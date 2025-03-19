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
 import frc.robot.Constants.IntakeConstants;

 public class CommandController extends SubsystemBase{
    //Definitions
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    PoseSubsystem poseSubsystem = new PoseSubsystem();
    ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    int setpointSelected;

    //Subsystem Method
    public CommandController(){
    }

    //Reference Methods
    public Command Path(){

    }

    public Command CancelPath(){

    }

    public void PIDSuperCommand(double intakeSetpoint, double elevatorSetpoint){
        //Initial safety arm PID with intentional overshoot.
        if (intakeSubsystem.getArmEncoder() > IntakeConstants.kIntakeSafeEncoderValue){
            Commands.runOnce(() -> intakeSubsystem.SetGoal(IntakeConstants.kIntakeSafeEncoderValueAdjusted), intakeSubsystem).andThen
            (Commands.run(() -> intakeSubsystem.gotoPosition(), intakeSubsystem)).until(() -> intakeSubsystem.AtGoal(intakeSetpoint)).andThen
            (Commands.runOnce(() -> intakeSubsystem.setAntiGravity(), intakeSubsystem));
        }
        
        /* Either the arm's setpoint is within the safe range, or it's outside. In either case, the elevator can
        * immediately run while the arm will have to wait for the elevator reach its setpoint if it's outisde
        * of the safe range.
        */
        if (intakeSetpoint < IntakeConstants.kIntakeSafeEncoderValue){
            //Within safe zone
            Commands.runOnce(() -> intakeSubsystem.SetGoal(intakeSetpoint), intakeSubsystem).alongWith
            (Commands.runOnce(() -> elevatorSubsystem.SetGoal(elevatorSetpoint), elevatorSubsystem)).andThen
            (Commands.run(() -> intakeSubsystem.gotoPosition(), intakeSubsystem)).until(() -> intakeSubsystem.AtGoal(intakeSetpoint)).alongWith
            (Commands.run(() -> elevatorSubsystem.gotoPosition(), elevatorSubsystem)).until(() -> elevatorSubsystem.atGoal()).andThen
            (Commands.runOnce(() -> elevatorSubsystem.setAntiGravity(), elevatorSubsystem)).alongWith
            (Commands.runOnce(() -> elevatorSubsystem.setAntiGravity(), elevatorSubsystem));
        }else{
            //Outside safe zone
            Commands.runOnce(() -> intakeSubsystem.SetGoal(intakeSetpoint), intakeSubsystem).alongWith
            (Commands.runOnce(() -> elevatorSubsystem.SetGoal(elevatorSetpoint), elevatorSubsystem)).andThen
            (Commands.run(() -> elevatorSubsystem.gotoPosition(), elevatorSubsystem)).until(() -> elevatorSubsystem.atGoal()).andThen
            (Commands.run(() -> intakeSubsystem.gotoPosition(), intakeSubsystem)).until(() -> intakeSubsystem.AtGoal(intakeSetpoint)).andThen
            (Commands.runOnce(() -> elevatorSubsystem.setAntiGravity(), elevatorSubsystem)).alongWith
            (Commands.runOnce(() -> elevatorSubsystem.setAntiGravity(), elevatorSubsystem));
        }
    }

    public void ElevatorFineTune(double input){
        Commands.run(() -> elevatorSubsystem.ElevatorFineTune(input),elevatorSubsystem);
    }

    public void zeroEleEncoderNoCommands(double input){
        Commands.run(() -> intakeSubsystem.ArmFineTune(input),elevatorSubsystem);
    }

    public void ArmFineTune(double input){
        Commands.run(() -> intakeSubsystem.ArmFineTune(input),elevatorSubsystem);    }

    public void Intake(){
        Commands.runOnce(() -> intakeSubsystem.Intake());
    }
    
    public void Shoot(){
        Commands.runOnce(() -> intakeSubsystem.Shoot());
    }

    public void Climb(double polarity){
        Commands.run(() -> ClimbSubsystem.Climb(NeoMotorConstants.kClimberSpeed * polarity));
    }

    public void StopClimb(){
       Commands.runOnce(() -> ClimbSubsystem.StopClimb());
    }
 }
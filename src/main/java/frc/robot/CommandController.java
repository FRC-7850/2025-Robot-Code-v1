 /*
 *This file will hold all of the command methods and also orchestrate commands to run
 *after auto-pathing is completed.
 */

 package frc.robot;

//WPILib
 import edu.wpi.first.wpilibj2.command.Commands;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

    //File structure
 //Subsystems
 import frc.robot.subsystems.ElevatorSubsystem;
 import frc.robot.subsystems.PoseSubsystem;
 import frc.robot.subsystems.IntakeSubsystem;
 import frc.robot.subsystems.ClimbSubsystem;
 //Constants
 import frc.robot.Constants.NeoMotorConstants;
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
    // public Command Path(){

    // }

    // public Command CancelPath(){

    // }

    /**
     * Run Arm and Elevator PID, with safety mechanisms. 
     * @param intakeSetpoint double, (from constants)
     * @param elevatorSetpoint double, (from constants)
     */
    public void PIDSuperCommand(double intakeSetpoint, double elevatorSetpoint){
        //Initial safety arm PID with intentional overshoot.
        if (intakeSubsystem.getArmEncoder() > IntakeConstants.kIntakeSafeEncoderValue){
            Commands.sequence(
                Commands.runOnce(() -> intakeSubsystem.SetGoal(IntakeConstants.kIntakeSafeEncoderValueAdjusted), intakeSubsystem),
                //Idle to ensure command can't end until arm has reached goal
                Commands.idle(intakeSubsystem)
                .until(() -> intakeSubsystem.AtGoal()));
        }
        
        /* Either the arm's setpoint is within the safe range, or it's outside. In either case, the elevator can
        * immediately run while the arm will have to wait for the elevator reach its setpoint if it's outisde
        * of the safe range.
        */
        if (intakeSetpoint < IntakeConstants.kIntakeSafeEncoderValue){
            //Within safe zone
            Commands.race(
                Commands.runOnce(() -> intakeSubsystem.SetGoal(intakeSetpoint), intakeSubsystem),
                Commands.runOnce(() -> elevatorSubsystem.SetGoal(elevatorSetpoint), elevatorSubsystem)
            );
        }else{
            Commands.sequence(
                Commands.runOnce(() -> elevatorSubsystem.SetGoal(elevatorSetpoint), elevatorSubsystem),
                //Idle to ensure command can't end until arm has reached goal
                Commands.idle(elevatorSubsystem)
                .until(() -> elevatorSubsystem.AtGoal()),
                Commands.runOnce(() -> intakeSubsystem.SetGoal(intakeSetpoint), intakeSubsystem)
            );
        }
    }

    public void Intake(double status){
        Commands.runOnce(() -> intakeSubsystem.Intake());
    }
    
    public void Shoot(double status){
        Commands.runOnce(() -> intakeSubsystem.Shoot());
    }

    public void Climb(double polarity){
        Commands.runOnce(() -> ClimbSubsystem.Climb(NeoMotorConstants.kClimberSpeed * polarity));
    }
 }
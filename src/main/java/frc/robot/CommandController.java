 /*
 *This file will hold all of the command methods and also orchestrate commands to run
 *after auto-pathing is completed.
 */

 package frc.robot;

 import java.security.Policy;

//WPILib
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

    //File structure
 //Subsystems
 import frc.robot.subsystems.ElevatorSubsystem;
 import frc.robot.subsystems.PoseSubsystem;
 import frc.robot.subsystems.IntakeSubsystem;
 import frc.robot.subsystems.ClimbSubsystem;
 //Constants
 import frc.robot.Constants.NeoMotorConstants;

 public class CommandController extends SubsystemBase{

    public CommandController(){
    }

    //Teleop Commands
    public Command Climb(double polarity){
        return this.runOnce(() -> ClimbSubsystem.Climb(NeoMotorConstants.kClimberSpeed * polarity));
    }

    public Command StopClimb(){
        return this.runOnce(() -> ClimbSubsystem.StopMotor());
    }
    
    public Command Intake(){
        //Serves as both an intake and shoot command
        return this.runOnce(() -> IntakeSubsystem.Intake());
    }

    public Command Shoot(){
        //Serves as both an intake and shoot command
        return this.runOnce(() -> IntakeSubsystem.Shoot());
    }

    public Command PIDToPos(){
        //Serves as both an intake and shoot command
        return this.runOnce(() -> ElevatorSubsystem.setToHeight());
    }

 }
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
 import frc.robot.subsystems.IntakeSubsystem;
 //Constants
 import frc.robot.Constants.IntakeConstants;

 public class CommandController extends SubsystemBase{
    //Definitions
    IntakeSubsystem m_intakeSubsystem;
    ElevatorSubsystem m_elevatorSubsystem;

    //Subsystem Method
    public CommandController(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem){
        m_intakeSubsystem = intakeSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
    }

    /**
     * Run Arm and Elevator PID, with safety mechanisms. 
     * @param intakeSetpoint double, (from constants)
     * @param elevatorSetpoint double, (from constants)
     */
    public void PIDSuperCommand(double intakeSetpoint, double elevatorSetpoint){
        System.out.println("Got Here!!!!");
        //Initial safety arm PID with intentional overshoot.
        if (m_intakeSubsystem.getArmEncoder() > IntakeConstants.kIntakeSafeEncoderValue){
            System.out.println("Got To Safety!!!!");
            Commands.sequence(
                Commands.runOnce(() -> m_intakeSubsystem.setSetpointButton(IntakeConstants.kIntakeSafeEncoderValueAdjusted), m_intakeSubsystem),
                //Idle to ensure command can't end until arm has reached goal
                Commands.idle(m_intakeSubsystem)
                .until(() -> m_intakeSubsystem.AtGoal()));
        }
        
        /* Either the arm's setpoint is within the safe range, or it's outside. In either case, the elevator can
        * immediately run while the arm will have to wait for the elevator reach its setpoint if it's outisde
        * of the safe range.
        */
        if (intakeSetpoint < IntakeConstants.kIntakeSafeEncoderValue){
            System.out.println("Got To Final Stage 1!!!!");
            //Within safe zone
            Commands.race(
                Commands.runOnce(() -> m_intakeSubsystem.setSetpointButton(intakeSetpoint), m_intakeSubsystem),
                Commands.runOnce(() -> m_elevatorSubsystem.setSetpointButton(elevatorSetpoint), m_intakeSubsystem)
            );
        }else{
            System.out.println("Got To Final Stage 2!!!!");
            Commands.sequence(
                Commands.runOnce(() -> m_elevatorSubsystem.setSetpointButton(elevatorSetpoint), m_intakeSubsystem),
                //Idle to ensure command can't end until arm has reached goal
                Commands.idle()
                .until(() -> m_intakeSubsystem.AtGoal()),
                Commands.runOnce(() -> m_elevatorSubsystem.setSetpointButton(intakeSetpoint), m_intakeSubsystem)
            );
        }
    }

 }
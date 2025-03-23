// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SetPointConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.LEDs;

//import edu.wpi.first.wpilibj2.command.button.PS4Controller;
//import edu.wpi.first.wpilibj.PS4Controller;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final ClimbSubsystem m_robotClimber = new ClimbSubsystem();
  private final CommandController commandController = new CommandController(m_robotIntake, m_robotElevator);
  private final LEDs m_leds = new LEDs(m_robotElevator);

  // The robot controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperationsControllerPort);
  CommandJoystick m_operatorStation = new CommandJoystick(OIConstants.kButtonPanelControllerPort);
//   CommandJoystick m_operatorStation = new CommandJoystick(OIConstants.kButtonPanelControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure NamedCommands for PathPlanner
    NamedCommands.registerCommand("PidToL2", 
       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmL2Setpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorL2Setpoint)))
    );    
    NamedCommands.registerCommand("PidToL3",
    Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmL2Setpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorL2Setpoint)))
    );
    NamedCommands.registerCommand("PidToBackwardsBarge",
    Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
      .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
      .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorBargeBackwardSetpoint)))
      .andThen(Commands.waitUntil(() -> m_robotElevator.AtGoal()))
      .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmBargeBackwardSetpoint)))
    );
    NamedCommands.registerCommand("Intake", 
      Commands.runOnce(() -> m_robotIntake.RunIntake(1), m_robotIntake)
    );
    NamedCommands.registerCommand("IntakeStop", 
      Commands.runOnce(() -> m_robotIntake.RunIntake(0), m_robotIntake)
    );
    NamedCommands.registerCommand("Shoot", getAutonomousCommand());
    NamedCommands.registerCommand("ShootSlow", getAutonomousCommand());
    NamedCommands.registerCommand("ShootStop", getAutonomousCommand());


    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    //Drive Command
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kR2.value)
        .whileTrue(Commands.runOnce(
            () -> m_robotDrive.resetGyro(),
            m_robotDrive)); 

    //Elevator Fine-tune
    // m_operatorController.axisMagnitudeGreaterThan(1, 0.1).whileTrue(Commands.run(() -> m_robotElevator.RunElevator(-m_operatorController.getLeftY())));
    // m_operatorController.axisMagnitudeGreaterThan(1, 0.1).onFalse(Commands.runOnce(() -> m_robotElevator.setAntiGravity()));

    //Arm Fine-tune 
    // m_operatorController.axisMagnitudeGreaterThan(5, 0.2).whileTrue(Commands.run(() -> m_robotIntake.RunArm(-m_operatorController.getRightY() * 0.2)));
    // m_operatorController.axisMagnitudeGreaterThan(5, 0.2).onFalse(Commands.runOnce(() -> m_robotIntake.RunArm(0)));

      //Intake
    //Triggers
    m_operatorController.leftBumper().onTrue(Commands.runOnce(() -> m_robotIntake.RunIntake(.85)));
    m_operatorController.leftBumper().onFalse(Commands.runOnce(() -> m_robotIntake.RunIntake(0)));
    m_operatorController.leftTrigger().onTrue(Commands.runOnce(() -> m_robotIntake.RunIntake(-1)));
    m_operatorController.leftTrigger().onFalse(Commands.runOnce(() -> m_robotIntake.RunIntake(0)));
    //Half-Speed
    m_operatorController.button(7).onTrue(Commands.runOnce(() -> m_robotIntake.RunIntakePrecise(1)));
    m_operatorController.button(7).onFalse(Commands.runOnce(() -> m_robotIntake.RunIntakePrecise(0)));

    //Setpoint Controls
    m_operatorStation.button(SetPointConstants.kAlgaeOnFloorSetpointButton).onTrue(
      Commands.either(
        Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))
       .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))),
       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))), 
        ()-> m_robotIntake.atSafeZone()));
    m_operatorStation.button(SetPointConstants.kProcessorSetpointButton).onTrue(
      Commands.either(
        Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))
       .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))),
       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))), 
        ()-> m_robotIntake.atSafeZone()));
    m_operatorStation.button(SetPointConstants.kAlgaeOnCoralSetpointButton).onTrue(
      Commands.either(
        Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))
       .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))),
       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))), 
        ()-> m_robotIntake.atSafeZone()));
    m_operatorStation.button(SetPointConstants.kL2SetpointButton).onTrue(
      Commands.either(
        Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))
       .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))),
       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))), 
        ()-> m_robotIntake.atSafeZone()));
    m_operatorStation.button(SetPointConstants.kL3SetpointButton).onTrue(
      Commands.either(
        Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))
       .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))),
       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))), 
        ()-> m_robotIntake.atSafeZone()));
    m_operatorStation.button(SetPointConstants.kBargeForwardSetpointButton).onTrue(
      Commands.either(
        Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))
        .andThen(Commands.runOnce(() -> m_robotIntake.FlipBarge(true)))
       .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))),
       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))
         .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
         .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint))), 
        ()-> m_robotIntake.atSafeZone()));
    m_operatorStation.button(SetPointConstants.kBargeBackwardSetpointButton).onTrue(
      Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
      .andThen(Commands.runOnce(() -> m_robotIntake.FlipBarge(true)))
      .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
      .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorBargeBackwardSetpoint)))
      .andThen(Commands.waitUntil(() -> m_robotElevator.AtGoal()))
      .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmBargeBackwardSetpoint)))
    );
    m_operatorStation.button(SetPointConstants.kHomePositionButton).onTrue(
      Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
      .andThen(Commands.waitUntil(() -> m_robotIntake.atSafeZone()))
      .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorHomePositionSetpoint)))
      .andThen(Commands.waitUntil(() -> m_robotElevator.AtGoal()))
      .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmHomePositionSetpoint)))
    );

    //climber Controls
    m_operatorController.rightBumper().onTrue(Commands.runOnce(() -> m_robotClimber.Climb(1)));
    m_operatorController.rightBumper().onFalse(Commands.runOnce(() -> m_robotClimber.Climb(0)));
    m_operatorController.rightTrigger().onTrue(Commands.runOnce(() -> m_robotClimber.Climb(-1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("JustMove1Auto");
  }
}
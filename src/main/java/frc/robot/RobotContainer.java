// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SetPointConstants;
import frc.robot.Constants.TransformConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import java.util.List;
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
  // Manual vs. Pathing controls
  public boolean pathingSwitch = false;

  // The robot's subsystems  private final ElevatorSubsystem robotElevator = new ElevatorSubsystem();
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem robotElevator = new ElevatorSubsystem();
  private final IntakeSubsystem robotIntake = new IntakeSubsystem();
  private final LEDs leds = new LEDs(robotElevator);
  private final CommandController commandController = new CommandController();

  // The robot controllers
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperationsControllerPort);
//   CommandJoystick operatorStation = new CommandJoystick(OIConstants.kButtonPanelControllerPort);

  //Joystick Slew Rate Limiter
  SlewRateLimiter operatorFilter = new SlewRateLimiter(TransformConstants.kOperatorSlewRate);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                pathingSwitch,
                false),
            robotDrive));
  }

  /*Robot Button Mappings:
   * 🎮Drive Controller:
    * Left Joystick: Strafe
    * Right Joystick: Rotate
    * X: Wheel lock
    * Y: Barge Toggle
    * B: Hazard Lights
    * A: Reset Heading
    * Dpad up: Climber up
    * Dpad down: Climber down
   * 🎮Operator Controller:
    * Left Trigger: Shoot
    * Left Bumper: Intake
    * Left Joystick: Elevator Fine-tune
    * Dpad up: Arm Fine-tune up
    * Dpad down: Arm Fine-tune up
   * 🎮Custom Pad:
    * Button 1:
    * Button 2:
    * Button 3:
    * Button 4:
    * Button 5:
    * Button 6:
    * Button 7:
    */
  private void configureButtonBindings() {
    //Drive Controls
    new JoystickButton(driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> robotDrive.setX(),
            robotDrive));

    //Elevator Controls
      //Fine tune
      operatorController.axisMagnitudeGreaterThan(1, 0.1).whileTrue(commandController.ElevatorFineTune(operatorFilter.calculate(-operatorController.getLeftY())));
      operatorController.axisMagnitudeGreaterThan(1, 0.1).whileFalse(commandController.ElevatorFineTune(0));

    // Intake Controls (DEMO!)
      //Arm
        //Fine tune
        //TODO: which axis is the arm axis?
        operatorController.axisMagnitudeGreaterThan(4, 0.1).whileTrue(commandController.ElevatorFineTune(operatorFilter.calculate(-operatorController.getLeftY())));
        operatorController.axisMagnitudeGreaterThan(4, 0.1).whileFalse(commandController.ElevatorFineTune(0));
      //Intake
        operatorController.leftBumper().onTrue(commandController.Intake(1));
        operatorController.leftBumper().onFalse(commandController.Intake(0));

    //Setpoint Controls

    //Pathing Controls
     
    //Barge Controls
    driverController.povUp().whileTrue(commandController.Barge(1));
    driverController.povDown().whileTrue(commandController.Barge(-1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        robotDrive::setModuleStates,
        robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, pathingSwitch, false));
  }




}
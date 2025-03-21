// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import frc.robot.subsystems.ClimbSubsystem;
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
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final ClimbSubsystem m_robotClimber = new ClimbSubsystem();
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

  public void SetPids(double armSetpoint, double elevatorSetpoint){
    // Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint), m_robotIntake)
    // .andThen(Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    // // .andThen(Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake))
    // .andThen(Commands.runOnce(() -> m_robotElevator.setSetpointButton(elevatorSetpoint), m_robotElevator))
    // .andThen(Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    // .andThen(Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator))
    // .andThen(Commands.runOnce(() -> m_robotIntake.setSetpointButton(armSetpoint), m_robotIntake))
    // .andThen(Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    // // .andThen(Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //Drive Command
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

            // m_operatorController.y().onTrue(Commands.runOnce(
            //     () -> m_robotIntake.zeroArmEncoder(),m_robotIntake));

    //Elevator Controls
    // m_operatorController.povDown().onTrue(Commands.runOnce(() -> m_robotElevator.RunElevator(-1)));
    // m_operatorController.povUp().onTrue(Commands.runOnce(() -> m_robotElevator.RunElevator(1)));
    // m_operatorController.povDown().onFalse(Commands.runOnce(() -> m_robotElevator.RunElevator(0)));
    // m_operatorController.povUp().onFalse(Commands.runOnce(() -> m_robotElevator.RunElevator(0)));
    // m_operatorController.axisGreaterThan(1, 0.1).onTrue(Commands.runOnce(() -> m_robotElevator.RunElevator(-1)));
    // m_operatorController.axisLessTHan(1, 0.1).onTrue(Commands.runOnce(() -> m_robotElevator.RunElevator(-1)));
    m_operatorController.axisMagnitudeGreaterThan(1, 0.1).whileTrue(Commands.run(() -> m_robotElevator.RunElevator(-m_operatorController.getLeftY())));
    //m_operatorController.axisMagnitudeGreaterThan(1, 0.1).onTrue(Commands.runOnce(() -> m_robotElevator.RunElevator(0)));
    m_operatorController.axisMagnitudeGreaterThan(1, 0.1).onFalse(Commands.runOnce(() -> m_robotElevator.setAntiGravity()));
    // m_operatorController.axisMagnitudeGreaterThan(1, 0.1).whileTrue(Commands.run(() -> m_robotElevator.RunElevator(m_operatorController.getLeftY())));
    // m_operatorController.axisMagnitudeGreaterThan(1, 0.1).onFalse(Commands.runOnce(() -> m_robotElevator.RunElevator(0)));
    
    //Disabled for Week Zero
    // m_operatorController.a().onTrue(Commands.runOnce(() -> m_robotElevator.setToHeight()));

    // Intake Controls (DEMO!)
    /*m_operatorController.povUp().onTrue(Commands.runOnce(() -> m_robotIntake.RunArm(1)));
    m_operatorController.povUp().onFalse(Commands.runOnce(() -> m_robotIntake.RunArm(0)));
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> m_robotIntake.RunArm(-1)));
    m_operatorController.povDown().onFalse(Commands.runOnce(() -> m_robotIntake.RunArm(0)));*/

    m_operatorController.axisMagnitudeGreaterThan(5, 0.2).whileTrue(Commands.run(() -> m_robotIntake.RunArm(-m_operatorController.getRightY() * 0.2)));
    //m_operatorController.axisMagnitudeGreaterThan(1, 0.1).onTrue(Commands.runOnce(() -> m_robotElevator.RunElevator(0)));
    m_operatorController.axisMagnitudeGreaterThan(5, 0.2).onFalse(Commands.runOnce(() -> m_robotIntake.RunArm(0)));

    m_operatorController.leftBumper().onTrue(Commands.runOnce(() -> m_robotIntake.RunIntake(.85)));
    //m_operatorController.rightBumper().onTrue(Commands.runOnce(() ->, null));
    m_operatorController.leftBumper().onFalse(Commands.runOnce(() -> m_robotIntake.RunIntake(0)));
    m_operatorController.leftTrigger().onTrue(Commands.runOnce(() -> m_robotIntake.RunIntake(-1)));
    m_operatorController.leftTrigger().onFalse(Commands.runOnce(() -> m_robotIntake.RunIntake(0)));

    //Setpoint Controls
      // m_operatorController.a().onTrue(Commands.run(() -> m_robotIntake.gotoPosition()));
      // m_operatorController.x().onTrue(Commands.runOnce(() -> m_robotIntake.setSetpoint()));
      // m_operatorController.y().onTrue(Commands.runOnce(() -> m_robotElevator.setSetpoint()));
      /*m_operatorController.x().onTrue(Commands.runOnce(() -> m_robotIntake.setAntiGravity()));
    m_operatorController.b().onTrue(Commands.runOnce(() -> m_robotIntake.setAntiGravityOff()));*/

      // m_operatorController.b().onTrue(Commands.run(() -> m_robotElevator.gotoPosition()).until(()->m_robotElevator.PIDAtGoal()).andThen(Commands.runOnce(()->m_robotElevator.setAntiGravity(), m_robotElevator)));

    // m_operatorStation.button(SetPointConstants.kAlgaeOnFloorSetpointButton).onTrue(
    //   //Safety (set arm straight out)
    //   Commands.run(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint))
    //   );
    // m_operatorStation.button(SetPointConstants.kBargeBackwardSetpointButton).onTrue(
    //     Commands.run(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmBargeBackwardSetpoint))
    //   );
    //   m_operatorStation.button(SetPointConstants.kAlgaeOnFloorSetpointButton).onTrue(
    //   //Safety (set arm straight out)
    //   Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
    //   .andThen(
    //     Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal())
    //     .andThen(
    //       Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //     )
    //     .andThen(
    //       Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorFloorSetpoint), m_robotElevator))
    //       .andThen(
    //         Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    //          .andThen(
    //           Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator)
    //          )
    //          .andThen(
    //           Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmFloorSetpoint), m_robotElevator))
    //           .andThen(
    //             Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    //             .andThen(
    //               Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //             )
    //     )
    //   );
    //   m_operatorStation.button(SetPointConstants.kAlgaeOnCoralSetpointButton).onTrue(
    //       //Safety (set arm straight out)
    //       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
    //       .andThen(
    //         Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal())
    //         .andThen(
    //           Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //         )
    //         .andThen(
    //           Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorAlgaeOnCoralSetpoint), m_robotElevator))
    //           .andThen(
    //             Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    //              .andThen(
    //               Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator)
    //              )
    //              .andThen(
    //               Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmAlgaeOnCoralSetpoint), m_robotElevator))
    //               .andThen(
    //                 Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    //                 .andThen(
    //                   Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                 )
    //         )
    //       );
    //       m_operatorStation.button(SetPointConstants.kL2SetpointButton).onTrue(
    //         //Safety (set arm straight out)
    //         Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
    //         .andThen(
    //           Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal())
    //           .andThen(
    //             Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //           )
    //           .andThen(
    //             Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorL2Setpoint), m_robotElevator))
    //             .andThen(
    //               Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    //                .andThen(
    //                 Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator)
    //                )
    //                .andThen(
    //                 Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmL2Setpoint), m_robotElevator))
    //                 .andThen(
    //                   Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    //                   .andThen(
    //                     Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                   )
    //           )
    //         );
    //         m_operatorStation.button(SetPointConstants.kL3SetpointButton).onTrue(
    //           //Safety (set arm straight out)
    //           Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
    //           .andThen(
    //             Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal())
    //             .andThen(
    //               Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //             )
    //             .andThen(
    //               Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorL3Setpoint), m_robotElevator))
    //               .andThen(
    //                 Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    //                  .andThen(
    //                   Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator)
    //                  )
    //                  .andThen(
    //                   Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmL3Setpoint), m_robotElevator))
    //                   .andThen(
    //                     Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    //                     .andThen(
    //                       Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                     )
    //             )
    //           );
    //           m_operatorStation.button(SetPointConstants.kBargeForwardSetpointButton).onTrue(
    //             //Safety (set arm straight out)
    //             Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
    //             .andThen(
    //               Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal())
    //               .andThen(
    //                 Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //               )
    //               .andThen(
    //                 Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorBargeForwardSetpoint), m_robotElevator))
    //                 .andThen(
    //                   Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    //                    .andThen(
    //                     Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator)
    //                    )
    //                    .andThen(
    //                     Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmBargeForwardSetpoint), m_robotElevator))
    //                     .andThen(
    //                       Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    //                       .andThen(
    //                         Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                       )
    //               )
    //             );
    //             m_operatorStation.button(SetPointConstants.kBargeBackwardSetpointButton).onTrue(
    //               //Safety (set arm straight out)
    //               Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
    //               .andThen(
    //                 Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal())
    //                 .andThen(
    //                   Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                 )
    //                 .andThen(
    //                   Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorBargeBackwardSetpoint), m_robotElevator))
    //                   .andThen(
    //                     Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    //                      .andThen(
    //                       Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator)
    //                      )
    //                      .andThen(
    //                       Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmBargeBackwardSetpoint), m_robotElevator))
    //                       .andThen(
    //                         Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    //                         .andThen(
    //                           Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                         )
    //                 )
    //               );
    //               m_operatorStation.button(SetPointConstants.kHomePositionButton).onTrue(
    //                 //Safety (set arm straight out)
    //                 Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kArmOutSetpoint))
    //                 .andThen(
    //                   Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal())
    //                   .andThen(
    //                     Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                   )
    //                   .andThen(
    //                     Commands.runOnce(() -> m_robotElevator.setSetpointButton(SetPointConstants.kElevatorHomePositionSetpoint), m_robotElevator))
    //                     .andThen(
    //                       Commands.run(() -> m_robotElevator.gotoPosition(), m_robotElevator).until(() -> m_robotElevator.PIDAtGoal()))
    //                        .andThen(
    //                         Commands.runOnce(() -> m_robotElevator.setAntiGravity(), m_robotElevator)
    //                        )
    //                        .andThen(
    //                         Commands.runOnce(() -> m_robotIntake.setSetpointButton(SetPointConstants.kElevatorHomePositionSetpoint), m_robotElevator))
    //                         .andThen(
    //                           Commands.run(() -> m_robotIntake.gotoPosition(), m_robotIntake).until(() -> m_robotIntake.PIDAtGoal()))
    //                           .andThen(
    //                             Commands.runOnce(() -> m_robotIntake.setAntiGravity(), m_robotIntake)
    //                           )
    //                   )
    //                 );

    //climber Controls
    m_operatorController.rightBumper().onTrue(Commands.runOnce(() -> m_robotClimber.Climb(1)));
    m_operatorController.rightBumper().onFalse(Commands.runOnce(() -> m_robotClimber.Climb(0)));
    m_operatorController.rightTrigger().onTrue(Commands.runOnce(() -> m_robotClimber.Climb(-1)));
  //  m_operatorController.rightTrigger().onFalse(Commands.runOnce(() -> m_robotClimber.Climb(0)));

  //Setpoint w/buttons
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
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
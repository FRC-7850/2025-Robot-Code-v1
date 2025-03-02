// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Java
import java.util.List;

//WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance; //TODO What's this for?
import edu.wpi.first.wpilibj.PS4Controller.Button; //Used for the drive command, though i think that's fucking stupid
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

//PathPlanner
import com.pathplanner.lib.commands.PathPlannerAuto;

  //File Structure
//Constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SetPointConstants;
import frc.robot.Constants.TransformConstants;
//Subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;

public class RobotContainer {
  //Shuffleboard Entries
  ShuffleboardTab auto = Shuffleboard.getTab("Autonomous");
  GenericEntry selectAuto;

    //Definitions
  //Subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final CommandController commandController = new CommandController();
  //Controllers
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperationsControllerPort); 
  CommandJoystick operatorStation = new CommandJoystick(OIConstants.kButtonPanelControllerPort);
  //Misc
  SlewRateLimiter operatorFilter = new SlewRateLimiter(TransformConstants.kOperatorSlewRate);
  public boolean pathingSwitch = false;

  //File Method
  public RobotContainer() {
    //Shuffleboard Entry Creation
    selectAuto = auto.add("Autonomous Selected", 0).withWidget(BuiltInWidgets.kComboBoxChooser).getEntry();

    //Initialize
    configureButtonBindings();
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

    // Intake Controls
      //Arm
        //Fine tune
        //TODO which axis is the arm axis?
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

  public Command getAutonomousCommand() {
    // return new InstantCommand(null);

    return new PathPlannerAuto("TestingShooterAuto");
  }
}
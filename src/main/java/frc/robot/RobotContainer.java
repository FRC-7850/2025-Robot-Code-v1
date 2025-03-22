// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 /*
 * "Theseus' Robot" Reefscape Master Code, Team 7850: C.A.R.D.S
 * Special thanks to 2181 for their incredible aid and mentorship during this season.
 */

package frc.robot;

//Java

//WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//PathPlanner
import com.pathplanner.lib.commands.PathPlannerAuto;

  //File Structure
//Constants
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SetPointConstants;
import frc.robot.Constants.TransformConstants;
//Subsystems
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  //Shuffleboard Entries
  ShuffleboardTab auto = Shuffleboard.getTab("Autonomous");
    //Definitions
  //Subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final CommandController commandController = new CommandController();
  //Controllers
  CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperationsControllerPort); 
  CommandJoystick operatorStation = new CommandJoystick(OIConstants.kButtonPanelControllerPort);
   //Misc
   SlewRateLimiter operatorFilter = new SlewRateLimiter(TransformConstants.kOperatorSlewRate);
   public boolean pathingSwitch = false;
   //Auto Chooser
   SendableChooser<String> autoChooser;

  //File Method
  public RobotContainer() {
    //Chooser definitions
    autoChooser.addOption("ExampleAuto1", "ExampleAutoName1");
    autoChooser.addOption("ExampleAuto2", "ExampleAutoName2");
    //Add to Shuffleboard
    Shuffleboard.getTab("Autonomous").add("Select Auto", autoChooser);

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
   * 🎮Driver Controller:
    * Left Joystick: Strafe
    * Right Joystick: Rotate
    * A: Reset Heading
   * 🎮Operator Controller:
    * Left Trigger: Shoot
    * Left Bumper: Intake
    * Right Trigger: Climb in
    * Right Bumper: Climb out
   * 🎮Operator Pad:
    * Button 1: 
    * Button 2:
    * Button 3:
    * Button 4:
    * Button 5:
    * Button 6:
    * Button 7:
    * Button 8:
   */
  private void configureButtonBindings() {
    //Wheel Lock
    driverController.x().whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));

    //Barge Controls
      //Up
      driverController.rightTrigger().onTrue(Commands.runOnce(() -> commandController.Climb(1), commandController));
      driverController.rightTrigger().onFalse(Commands.runOnce(() -> commandController.Climb(0), commandController));
      //Down
      driverController.rightBumper().onTrue(Commands.runOnce(() -> commandController.Climb(-1), commandController));
      driverController.rightBumper().onFalse(Commands.runOnce(() -> commandController.Climb(0), commandController));

    // Intake Controls
      //In
      operatorController.leftBumper().onTrue(Commands.runOnce(() -> commandController.Intake(1), commandController));
      operatorController.leftBumper().onFalse(Commands.runOnce(() -> commandController.Intake(0), commandController));
      //Out
      driverController.leftBumper().onTrue(Commands.runOnce(() -> commandController.Shoot(1), commandController));
      driverController.leftBumper().onFalse(Commands.runOnce(() -> commandController.Shoot(0), commandController));

    //Setpoint Controls
      operatorStation.button(SetPointConstants.kFloorSetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorFloorSetpoint,
          SetPointConstants.kArmFloorSetpoint
        ), commandController));

      operatorStation.button(SetPointConstants.kLollipopSetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorAlgaeOnCoralSetpoint,
          SetPointConstants.kArmAlgaeOnCoralSetpoint
        ), commandController));
        
      operatorStation.button(SetPointConstants.kProcessorSetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorProcessorSetpoint,
          SetPointConstants.kArmProcessorSetpoint
        ), commandController));

      operatorStation.button(SetPointConstants.kL2SetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorL2Setpoint,
          SetPointConstants.kArmL2Setpoint
        ), commandController));

      operatorStation.button(SetPointConstants.kL3SetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorL3Setpoint,
          SetPointConstants.kArmL3Setpoint
        ), commandController));

      operatorStation.button(SetPointConstants.kBargeForwardSetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorBargeForwardSetpoint,
          SetPointConstants.kArmFloorSetpoint
        ), commandController));
        
      operatorStation.button(SetPointConstants.kBargeBackwardSetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorBargeBackwardSetpoint,
          SetPointConstants.kArmFloorSetpoint
        ), commandController));

      operatorStation.button(SetPointConstants.kHomeSetpointButton).onTrue
        (Commands.runOnce(() -> commandController.PIDSuperCommand(
          SetPointConstants.kElevatorFloorSetpoint,
          SetPointConstants.kArmFloorSetpoint
        ), commandController));

      

    //Pathing Controls
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(null);
    // return new PathPlannerAuto(selectAuto.getString(null));
  }
}
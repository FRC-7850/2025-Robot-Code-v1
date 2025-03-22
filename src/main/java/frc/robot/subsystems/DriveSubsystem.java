// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.AutoConstants;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  //Shuffleboard 
  ShuffleboardTab DriveDebugTab = Shuffleboard.getTab("DriveDebugTab");
  GenericEntry M1Theta, M2Theta, M3Theta, M4Theta, M1Speed, M2Speed, M3Speed, M4Speed;

  //Create reference to Autobuilder class
  AutoBuilder m_AutoBuilder = new AutoBuilder();

  //PathPlanner configs
  RobotConfig config;

  //Drive Controller for Path Planner
  PPHolonomicDriveController autoDriveController = new PPHolonomicDriveController(
    AutoConstants.kAutoTranslationPIDConstants,
    AutoConstants.kAutoRotationPIDConstants
  );
  
  //Gobal reference to ChassisSpeeds object for Pathplanner
  SwerveModuleState[] swerveModuleStates;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroId);
  private StatusSignal<Angle> yaw = m_gyro.getYaw();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      new ChassisSpeeds(0,0,0)
    );
     
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    //Reference to pathplanner config. Pasted from docs.
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Configure Pathplanner. Some copy/paste code. 
    m_AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getChassisSpeeds, 
      this::driveRobotRelative, 
      autoDriveController, 
      config, 
      () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
      this);

      M1Theta = DriveDebugTab.add("Mod 1 Rotation", 0).getEntry();
      M2Theta = DriveDebugTab.add("Mod 2 Rotation", 0).getEntry();
      M3Theta = DriveDebugTab.add("Mod 3 Rotation", 0).getEntry();
      M4Theta = DriveDebugTab.add("Mod 4 Rotation", 0).getEntry();
      M1Speed = DriveDebugTab.add("Mod 1 Speed", 0).getEntry();
      M2Speed = DriveDebugTab.add("Mod 2 Speed", 0).getEntry();
      M3Speed = DriveDebugTab.add("Mod 3 Speed", 0).getEntry();
      M4Speed = DriveDebugTab.add("Mod 4 Speed", 0).getEntry();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    M1Theta.setDouble(m_frontLeft.getState().angle.getRadians());
    M2Theta.setDouble(m_frontRight.getState().angle.getRadians());
    M3Theta.setDouble(m_rearLeft.getState().angle.getRadians());
    M4Theta.setDouble(m_rearLeft.getState().angle.getRadians());
    M1Speed.setDouble(m_frontLeft.getState().speedMetersPerSecond);
    M2Speed.setDouble(m_frontRight.getState().speedMetersPerSecond);
    M3Speed.setDouble(m_rearLeft.getState().speedMetersPerSecond);
    M4Speed.setDouble(m_rearLeft.getState().speedMetersPerSecond);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds speeds = 
    fieldRelative? 
    ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeedDelivered, 
      ySpeedDelivered, 
      rotDelivered, 
      Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble()))
    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().refresh().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //Created for Pathplanner integration
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void resetGyro(){
    m_gyro.reset();
  }
}

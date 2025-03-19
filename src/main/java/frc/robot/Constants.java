// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CanIDConstants{
    //Drive
      //Driving Motors
      public static final int kFrontLeftDrivingCanId = 3;
      public static final int kRearLeftDrivingCanId = 5;
      public static final int kFrontRightDrivingCanId = 1;
      public static final int kRearRightDrivingCanId = 7;
      //Turning Motors
      public static final int kFrontLeftTurningCanId = 4;
      public static final int kRearLeftTurningCanId = 6;
      public static final int kFrontRightTurningCanId = 2;
      public static final int kRearRightTurningCanId = 8;
      //Gyro
      public static final int kGyroId = 9;

    //Elevator
    public static final int kElevatorLeftCanId = 20;
    public static final int kElevatorRightCanId = 21;

    //Arm
    public static final int kArmLeftCanId = 16;
    public static final int kArmRightCanId = 55;

    //Intake
    public static final int kIntakeLeftCanId = 40;
    public static final int kIntakeRightCanId = 41;

    //Climb
    public static final int kClimberCanId = 50;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        // new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        // new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        // new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperationsControllerPort = 1;
    public static final int kButtonPanelControllerPort = 2;

    public static final double kDriveDeadband = 0.1;;

    //Arm/Intake (UNSET)
  }

  public static final class LEDConstants {

    public static final int LEDPWM = 0;
    public static final int NUMLED = 150;
    public static final int BRIGHTPERCENT = 50;
    public static final int TEAMRGB1[] = {255,0,0};
    public static final int TEAMRGB2[] = {255,255,255};
    public static final int UNDERGLOWSTART = 53;
    public static final int UNDERGLOWEND = 55;
    public static final int ELEVATORSTART = 0;
    public static final int ELEVATOREND = 52;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;

    public static final double kElevatorTimeToInterpolate = 0.5;
    public static final double kArmTimeToInterpolate = 0.5;
    public static final double kClimberTimeToInterpolate = 0.5;

    public static final double kClimberSpeed = 0.75;
  }

  public static final class TransformConstants{
    //Slew Rate Limit
    public static final double kOperatorSlewRate = 0.5;

    //Vision
    public static final double kCameraToRobotOffsetX = .1; //Axis through front
    public static final double kCameraToRobotOffsetY = .1; //Axis through sides
    public static final double kCameraToRobotOffsetZ = .1; //Vertical axis, will not be constant.
    public static final Rotation3d kCameraToRobotOffsetTheta = new Rotation3d(0,0,0); //0,0,0 implies facing straight forward and level with bellypan
    public static final Transform3d kCameraToRobot = new Transform3d(kCameraToRobotOffsetX, kCameraToRobotOffsetY, kCameraToRobotOffsetZ, kCameraToRobotOffsetTheta);
  }

  public static final class TagGoals{
  }

  public static final class ElevatorConstants{
    public static final int kUpButton = 2;
    public static final int kDownButton = 1;
    public static final double kElevatorMaxSpeed = .5;
    public static final double kS = 0, kG = 0, kV = 0, kA = 0, kP = 0, kI = 0, kD = 0; //TODO
    public static final double kElevatorMaxHeight = 120.0;
    public static final double kElevatorMinOutput = 1;
    public static final double kElevatorMaxOutput = 1;
    public static double kMaxEleSP; //TODO
    public static double kMinEleSP; //TODO
    public static double kArmFineTuneSpeed = 0.1; //TODO
  }

  public static final class IntakeConstants{
    public static final int kUpButton = 2;
    public static final int kDownButton = 1;
    public static final double kIntakeSpeed = 85;
    public static final double kArmFineTuneSpeed = .5;
    public static final double kIntakeSafeEncoderValue = 0; //TODO
    public static final double kIntakeSafeEncoderValueAdjusted = 0; //TODO
    public static final double kS = 0, kG = 0, kV = 0, kA = 0, kP = 0, kI = 0, kD = 0; //TODO: Fill in

  }

  public static final class SetPointConstants{
    //Driverstation Button -> Setpoint Bindings //TODO
    public static final int kBargeSetpointButton = 0;
    public static final int kProcessorSetpointButton = 1;
    public static final int kL3SetpointButton = 2;
    public static final int kL2SetpointButton = 3;
    public static final int kAlgaeOnCoralSetpointButton = 4;
    public static final int kAlgaeOnFloorSetpointButton = 5;

    //Setpoint Values (Unset)
      //Elevator //TODO
      public static final double kElevatorBargeSetpoint = 0;
      public static final double kElevatorProcessorSetpoint = 0;
      public static final double kElevatorL3Setpoint = 0;
      public static final double kElevatorL2Setpoint = 0;
      public static final double kElevatorAlgaeOnCoralSetpoint = 0;
      public static final double kElevatorFloorSetpoint = 0;
       //Arm //TODO
       public static final double kArmBargeSetpoint = 0;
       public static final double kArmProcessorSetpoint = 0;
       public static final double kArmL3Setpoint = 0;
       public static final double kArmL2Setpoint = 0;
       public static final double kArmAlgaeOnCoralSetpoint = 0;
       public static final double kArmFloorSetpoint = 0;
  }
}

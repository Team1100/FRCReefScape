// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.vision.VisionConfig;

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
  public static final class AlgaeIntakeConstants {
    //AlgaeIntake constants
    public static final boolean kEnableAnglePIDTuning = true;
    public static final double kAngleP = 0;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;

    public static final double kAngleSpeed = 0.2;
    public static final double kAngleSpeedRPM = 30;

    public static final double kAngleLowerLimitDegrees = 0;
    public static final double kAngleUpperLimitDegrees = 0;

    public static final double kAngleScorePositionDegrees = 0;
    public static final double kAngleIntakePositionDegrees = 0;

    public static final double kAngleEncoderPositionFactor = (2 * Math.PI);

    public static final boolean kEnableRollerPIDTuning = true;
    public static final double kRollerP = 0;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double kRollerSpeed = 0.2;
    public static final double kRollerSpeedRPM = 30;

    public static final double DEGREES_PER_REVOLUTION = 360;
  }

  public static final class WoSConstants {
    //WoS constants
    public static final boolean kEnableWheelPIDTuning = true;
    public static final double kWoSP = 0;
    public static final double kWoSI = 0;
    public static final double kWoSD = 0;

    public static final double kWoSSpeed = 0.2;
    public static final double kWoSSpeedRPM = 30;

    public static final boolean kEnableShoulderPIDTuning = true;
    public static final double kShoulderP = 0;
    public static final double kShoulderI = 0;
    public static final double kShoulderD = 0;
    public static final double kShoulderkS = 0;
    public static final double kShoulderkG = 0;
    public static final double kShoulderkV = 0;

    public static final double kWoSShoulderSpeed = 0.2;
    public static final double kWoSShoulderSpeedRPM = 30;
    
    public static final double kWoSShoulderEncoderPositionFactor = (2 * Math.PI);
    public static final double kWoSShoulderEncoderVelocityFactor = (2 * Math.PI / 60.0);

    public static final double DEGREES_PER_REVOLUTION = 360;
    public static final double kAngleLowerLimitDegrees = 0;
    public static final double kAngleUpperLimitDegrees = 0;

    public static final double[] kWoSShoulderLevels = {
      0,
      0,
      0,
      0
    };

    public static final double kEatAngle = 0;
  }

  public static final class ElevatorConstants {
    //Elevator Constants
    public static final boolean kEnableElevatorPIDTuning = true;
    public static final double kElevatorP = 0;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorkS = 0;
    public static final double kElevatorkG = 0;
    public static final double kElevatorkV = 0;

    public static final double kElevatorMaxVelocity = 0;
    public static final double kElevatorMaxAcceleration = 0;
    public static final double kElevatorSpeed = 0.2;
    public static final double kElevatorSpeedRPM = 30;

    public static final double kElevatorEncoderPositionFactor = (2 * Math.PI);

    public static final double DEGREES_PER_REVOLUTION = 360;
    public static final double kElevatorLowerLimitDegrees = 0;
    public static final double kElevatorUpperLimitDegrees = 0;

    public static final double[] kElevatorLevels = {
      0,
      0,
      0,
      0
    };
  }

  public static final class FunnelConstants {
    public static final boolean kEnableFunnelPIDTuning = true;
    public static final double kFunnelP = 0;
    public static final double kFunnelI = 0;
    public static final double kFunnelD = 0;

    public static final double kFunnelSpeed = 0.2;
    public static final double kFunnelSpeedRPM = 30;
  }

  public static final class LightsConstants {
    // Defines Lights constants
    public static final int LED_LENGTH = 42; // number of LEDs
  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
   // Locations for the swerve drive modules relative to the robot center.
    public static final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    // Creating my kinematics object using the module locations
    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs are in RobotMap

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
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 10;
    public static final double kIThetaController = 0.1;
    public static final double kDThetaController = 0.05;
    public static final double kThetaTolerance = 0.1;//radians

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // TODO: (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    
    // Maximum ambiguity accepted as a valid result from the vision systems
    public static final double kMaxValidAmbiguity = 0.2;

    // TODO: These values are from Mania! Must be determined for new robot...
    public static final VisionConfig[] kVisionSystems = {
        new VisionConfig("Arducam_OV9281_USB_Camera",
                         new Transform3d(new Translation3d(Units.inchesToMeters(15.5), Units.inchesToMeters(0.0), Units.inchesToMeters(21.5)), 
                                new Rotation3d(0, Units.degreesToRadians(180), Units.degreesToRadians(0))),
                         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                         PoseStrategy.LOWEST_AMBIGUITY),
        new VisionConfig("Arducam_OV2311_USB_Camera",
                         new Transform3d(new Translation3d(Units.inchesToMeters(15.75), Units.inchesToMeters(8.0), Units.inchesToMeters(8.0)), 
                new Rotation3d(0, Units.degreesToRadians(180), Units.degreesToRadians(0))),
                         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                         PoseStrategy.LOWEST_AMBIGUITY)
    };
  }

  public static final class ClimberConstants {
    public static final double kWinchP = 0;
    public static final double kWinchI = 0;
    public static final double kWinchD = 0;
    public static final boolean kEnableWinchPIDTuning = false;
  }

  //Driver control rate limits
  public static final double kMaxAccelerationMetersPerSecondSquared = 10;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;
  public static final double kMaxSpeedMetersPerSecond = 4.8;
  public static final double kMaxAngularSpeed = 2 * Math.PI;
  public static final double kWheelDiameterMeters = 0.0762;
  public static final double D_ANGLE_TOLERANCE_DEGREES = 2.5;
  public static final int kDrivingMotorPinionTeeth = 14;
  public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

  public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
    
    // Defines Neo Motor constant
    public static final double kFreeSpeedRpm = 5676;

     // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
   
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
    / kDrivingMotorReduction; // meters per second

    public static final double kDrivingP = 0.02;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    //Enables SysID Characterization Mode. !!Should be false during competitions. Can cause the Operator controller to be remapped!!
    public static final boolean kSysIdModeEnabled = false;
}
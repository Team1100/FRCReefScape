// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.utils.TargetPose;
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

    public static final double kAngleSpeed = 1;
    public static final double kAngleSpeedRPM = 3000;

    public static final double kAngleLowerLimitDegrees = 0;
    public static final double kAngleUpperLimitDegrees = 0;

    public static final double kAngleScorePositionDegrees = 0;
    public static final double kAngleIntakePositionDegrees = 0;

    public static final double kAngleEncoderPositionFactor = (2 * Math.PI);

    public static final boolean kEnableRollerPIDTuning = true;
    public static final double kRollerP = 0;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;

    public static final double kRollerSpeed = 1;
    public static final double kRollerSpeedRPM = 3000;

    public static final double DEGREES_PER_REVOLUTION = 360;
  }

  public static final class WoSConstants {
  //WoS constants
  public static final boolean kEnableWheelPIDTuning = true;
  public static final double kWoSP = 0;
  public static final double kWoSI = 0;
  public static final double kWoSD = 0;

  public static final double kWoSSpeed = 1;
  public static final double kWoSSpeedRPM = 3000;
  }

  public static final class ElevatorConstants {
    //Elevator Constants
    public static final boolean kEnableElevatorPIDTuning = true;
    public static final double kElevatorP = 0;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;

    public static final double kElevatorSpeed = 1;
    public static final double kElevatorSpeedRPM = 3000;
  }

  public static final class FunnelConstants {
    public static final boolean kEnableFunnelPIDTuning = true;
    public static final double kFunnelP = 0;
    public static final double kFunnelI = 0;
    public static final double kFunnelD = 0;

    public static final double kFunnelSpeed = 1;
    public static final double kFunnelSpeedRPM = 3000;
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

    public static final Translation2d[] m_ModulePositions = new Translation2d[] { m_frontRightLocation, m_frontLeftLocation, m_backRightLocation, m_backLeftLocation };

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

    // Distance from robot center to furthest module
    public static final double kBaseRadius = Units.inchesToMeters(RobotMap.R_BASE_RADIUS_INCHES);
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
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 10;
    public static final double kIThetaController = 0.1;
    public static final double kDThetaController = 0.05;
    public static final double kThetaTolerance = 0.1;//radians

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

    // AutoBuilder dynamic robot constriants
    public static final PIDConstants kPathFollowerTranslationPID = new PIDConstants(5.0, 0.0, 0.0); // Translation PID constants
    public static final PIDConstants kPathFollowerRotationPID = new PIDConstants(5.0, 0.0, 0.0); // Rotation PID constants    

    public static final double kPathFollowerMaxSpeed = Constants.kMaxSpeedMetersPerSecond; // Max module speed, in m/s
    public static final double kPathFollowerBaseRadius = DriveConstants.kBaseRadius; // Drive base radius in meters
    public static final double kPathFollowerMass = 52.1631; // 115 pounds
    public static final double kPathFollowerMomentOfInertia = 6.2; // Total guess. Rough estimate of l^2 + w^2 * m * 1/12
    public static final double kPathFollowerWheelCoeficientFriction = 1.2; // Total guess. pathplaner default

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


    public static final TargetPose kBlueReefAPose = new TargetPose(new Pose2d(3.111, 4.187, new Rotation2d(Units.degreesToRadians(0))));
    public static final TargetPose kBlueReefBPose = new TargetPose(new Pose2d(3.129, 3.809, new Rotation2d(Units.degreesToRadians(0))));
    public static final TargetPose kBlueCoralA1Pose = new TargetPose(new Pose2d(1.690, 7.334, new Rotation2d(Units.degreesToRadians(-50))), true);
    public static final TargetPose kBlueCoralA2Pose = new TargetPose(new Pose2d(0.773, 6.668, new Rotation2d(Units.degreesToRadians(-50))), true);

    public static final TargetPose kRedReefAPose = new TargetPose(new Pose2d(14.457, 3.809, new Rotation2d(Units.degreesToRadians(180))));
    public static final TargetPose kRedReefBPose = new TargetPose(new Pose2d(14.457, 4.187, new Rotation2d(Units.degreesToRadians(180))));
    public static final TargetPose kRedReefKPose = new TargetPose(new Pose2d(13.585, 2.730, new Rotation2d(Units.degreesToRadians(120))));
    public static final TargetPose kRedReefLPose = new TargetPose(new Pose2d(13.900, 2.910, new Rotation2d(Units.degreesToRadians(120))));
    public static final TargetPose kRedCoralA1Pose = new TargetPose(new Pose2d(15.878, 0.773, new Rotation2d(Units.degreesToRadians(125))), true);
    public static final TargetPose kRedCoralA2Pose = new TargetPose(new Pose2d(16.858, 1.382, new Rotation2d(Units.degreesToRadians(125))), true);
}
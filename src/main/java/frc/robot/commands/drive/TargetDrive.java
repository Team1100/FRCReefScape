// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.SwerveDriveInputs;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class TargetDrive extends Command {
    private Drive m_drive;
    private SwerveDriveInputs m_driveInputs;
    private Supplier<Pose2d> m_tgtSupplier;
    private ProfiledPIDController m_thetaController;
    
    private boolean atGoal = false;

    TDNumber TDCurrentAngle;
    TDNumber TDTargetAngle;
    TDNumber TDRotationFeedForward;
    TDNumber FFVel;
    TDNumber FFDif;
    TDSendable m_thetaControllerSendable;

  /** Creates a new TargetDrive. */
  public TargetDrive(Supplier<Pose2d> targetSupplier, SwerveDriveInputs driveInputs) {
    super(Drive.getInstance(), "", "TargetDrive");
    m_drive = Drive.getInstance();
    m_driveInputs = driveInputs;
    m_tgtSupplier = targetSupplier;

    m_thetaController = new ProfiledPIDController(
          Constants.AutoConstants.kPThetaController
        , 0
        , 0
        //, Constants.AutoConstants.kDThetaController
        , Constants.AutoConstants.kThetaControllerConstraints
      );
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_thetaController.setTolerance(0.1);

    m_thetaControllerSendable = new TDSendable(m_drive, "", "TargetDriveThetaController", m_thetaController);

    TDCurrentAngle = new TDNumber(m_drive, "Test Outputs", "Current Angle");
    TDTargetAngle = new TDNumber(m_drive, "Test Outputs", "Target Angle");
    TDRotationFeedForward = new TDNumber(m_drive, "Test Outputs", "Rot FF");
    FFVel = new TDNumber(m_drive, "Test Outputs", "FF Vel");
    FFDif = new TDNumber(m_drive, "Test Outputs", "FF Dif");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  public boolean atGoal() {
    return atGoal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_thetaController.reset(MathUtil.angleModulus(m_drive.getPose().getRotation().getRadians()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = m_tgtSupplier.get();
    Pose2d currentPose = m_drive.getPose();
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_drive.getMeasuredSpeeds(), currentPose.getRotation());

    Rotation2d targetRot = FieldUtils.getInstance().getAngleToPose(currentPose, targetPose);

    TDCurrentAngle.set(currentPose.getRotation().getDegrees());
    TDTargetAngle.set(targetRot.getDegrees());
    
    double rotation = m_thetaController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()),
            MathUtil.angleModulus(targetRot.getRadians()));
    double ff = calculateRotationFF(fieldRelativeSpeeds, currentPose, targetPose, targetRot);
    TDRotationFeedForward.set(ff);
    rotation += ff;
    
    double xSpeed =  -MathUtil.applyDeadband(m_driveInputs.getX(), Constants.OIConstants.kDriveDeadband) * Constants.kMaxSpeedMetersPerSecond;
    double ySpeed =  -MathUtil.applyDeadband(m_driveInputs.getY(), Constants.OIConstants.kDriveDeadband) * Constants.kMaxSpeedMetersPerSecond;
    Rotation2d fieldRotationOffset = currentPose.getRotation().plus(FieldUtils.getInstance().getRotationOffset());

    ChassisSpeeds outputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, fieldRotationOffset);
    m_drive.drive(outputSpeeds);

    atGoal = MathUtil.isNear(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees(), Constants.D_ANGLE_TOLERANCE_DEGREES);
    var traj = TrajectoryGenerator.generateTrajectory(List.of(currentPose, new Pose2d(targetPose.getTranslation(), currentPose.getRotation())), new TrajectoryConfig(5, 5));
    m_drive.m_field.getObject("traj").setTrajectory(traj);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculateRotationFF(ChassisSpeeds currentSpeeds, Pose2d currentPose, Pose2d targetPose, Rotation2d targetAngle) {

    Translation2d velocityTrans = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    var perpendicularVelocity = currentPose.getRotation().minus(targetAngle).getSin() * velocityTrans.getNorm();
    var output = perpendicularVelocity * (targetPose.minus(currentPose).getTranslation().getNorm()) * -1;
    FFDif.set(targetAngle.getDegrees());
    
    return output;
  }
}
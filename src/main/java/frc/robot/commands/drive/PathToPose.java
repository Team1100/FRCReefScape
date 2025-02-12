package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.Drive;
import frc.robot.utils.TargetPose;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.OI;

public class PathToPose extends Command {

  Supplier<TargetPose> m_targetPose;
  edu.wpi.first.wpilibj2.command.Command m_currentPathCommand;
  Pose2d m_currentTarget;
  Drive m_Drive;

  /** Creates a new DriveToPose. */
  public PathToPose(Supplier<TargetPose> poseSupplier) {
    m_Drive = Drive.getInstance();
    m_targetPose = poseSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TargetPose destination = m_targetPose.get();
    var destinationPose = destination.getPose();
    if (m_currentPathCommand == null || m_currentTarget == null || !m_currentTarget.equals(destinationPose)) {

      var rotations = new LinkedList<RotationTarget>();
      var waypoints = new LinkedList<Waypoint>();
      var currentRotation = Drive.getInstance().m_field.getRobotPose().getRotation();
      var currentLocation = Drive.getInstance().m_field.getRobotPose().getTranslation();
      var currentControlPoint = currentLocation.plus(new Translation2d(0.25, currentRotation.plus(new Rotation2d(Units.degreesToRadians(180)))));
      waypoints.add(new Waypoint(null, currentLocation, currentControlPoint));
      Rotation2d destinationApproachAngle;
      if (destination.getReversedApproach()) {
        destinationApproachAngle = destinationPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(180)));
      } else {
        destinationApproachAngle = destinationPose.getRotation();
      }
      var approachControlPoint = destinationPose.getTranslation().minus(new Translation2d(1, destinationApproachAngle));

      switch (OI.getInstance().getApproachType()) {
        case c_LeftApproach:
          approachControlPoint = destinationPose.getTranslation().minus(new Translation2d(1, destinationPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(90)))));
          waypoints.add(new Waypoint(approachControlPoint, destinationPose.getTranslation(), null));
          break;
        case c_RightApproach:
          approachControlPoint = destinationPose.getTranslation().minus(new Translation2d(1, destinationPose.getRotation().minus(new Rotation2d(Units.degreesToRadians(90)))));
          waypoints.add(new Waypoint(approachControlPoint, destinationPose.getTranslation(), null));
          break;
        case c_LeftSpin:
          rotations.add(new RotationTarget(0.90, new Rotation2d(Units.degreesToRadians(-90))));
          waypoints.add(new Waypoint(approachControlPoint, destinationPose.getTranslation(), null));
          break;
        case c_RightSpin:
          rotations.add(new RotationTarget(0.90, new Rotation2d(Units.degreesToRadians(90))));
          waypoints.add(new Waypoint(approachControlPoint, destinationPose.getTranslation(), null));
          break;
        case c_Straight:
        default:
          waypoints.add(new Waypoint(approachControlPoint, destinationPose.getTranslation(), null));
          break;
      }
     var constraints = new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond * 10,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared * 10,
      AutoConstants.kMaxAngularSpeedRadiansPerSecond * 10,
      AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared * 10);

      // Create the path using the waypoints created above
      var currentPath = new PathPlannerPath(
          waypoints,
          rotations,
          new ArrayList<PointTowardsZone>(), // PointTowardsZones
          new ArrayList<ConstraintsZone>(), // ConstraintZones
          new ArrayList<EventMarker>(), // EvevntMarkers
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                // be null for on-the-fly paths.
          new GoalEndState(0.0, destinationPose.getRotation()),
          false);

      // Prevent the path from being flipped if the coordinates are already correct
      currentPath.preventFlipping = true;
      m_currentPathCommand = new ProxyCommand(AutoBuilder.followPath(currentPath));
      m_currentPathCommand.initialize();
      m_currentTarget = destinationPose;
    }
    m_currentPathCommand.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_currentPathCommand != null) {
      m_currentPathCommand.end(interrupted);
      m_currentPathCommand = null;
    }
    m_currentTarget = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_currentPathCommand != null) {
      return m_currentPathCommand.isFinished();
    }
    return true;
  }

}

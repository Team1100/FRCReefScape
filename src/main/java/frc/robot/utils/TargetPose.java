package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetPose {
  protected final Pose2d pose;
  protected final boolean reversedApproach;

  public TargetPose(Pose2d pose, boolean reversedApproach){
    this.pose = pose;
    this.reversedApproach = reversedApproach;
  }

  public TargetPose(Pose2d pose){
    this.pose = pose;
    this.reversedApproach = false;
  }

  public Pose2d getPose() {
    return this.pose;
  }

  public boolean getReversedApproach() {
    return  this.reversedApproach;
  }
}

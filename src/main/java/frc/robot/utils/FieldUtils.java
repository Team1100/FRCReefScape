// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
 
public class FieldUtils {

    public enum ApproachType {
        c_LeftApproach,
        c_RightApproach,
        c_LeftSpin,
        c_RightSpin,
        c_Straight
    }
    
    private static FieldUtils m_fieldUtils;

    public static final AllianceAprilTags RedTags = new AllianceAprilTags(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    public static final AllianceAprilTags BlueTags = new AllianceAprilTags(12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22);

    public static FieldUtils getInstance(){
        if(m_fieldUtils == null){
            m_fieldUtils = new FieldUtils();
        }
        return m_fieldUtils;
    }

    private FieldUtils(){}

    public AllianceAprilTags getAllianceAprilTags(){
        AllianceAprilTags tags = BlueTags;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Red){
                tags = RedTags;
            } else if(alliance.get() == DriverStation.Alliance.Blue) {
                tags = BlueTags;
            }
        }
        return tags;
    }

    public Pose3d getTagPose(int tagId)
    {
        return Constants.VisionConstants.kTagLayout.getTagPose(tagId).get();
    }
    
    public Rotation2d getRotationOffset() {
        Rotation2d offset = new Rotation2d();//returns no offset by default
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && 
            alliance.get() == DriverStation.Alliance.Red){
                offset = new Rotation2d(Math.PI);
        }
        return offset;
    }

    public Rotation2d getAngleToPose(Pose2d currentPose, Pose2d targetPose) {
        Translation2d curTrans = currentPose.getTranslation();
        Translation2d targetTrans = targetPose.getTranslation();
        Translation2d toTarget = targetTrans.minus(curTrans);
        return toTarget.getAngle();
    }
    
    public TargetPose getBlueReefAPose() {
        return Constants.kBlueReefAPose;
    }

    public TargetPose getBlueReefBPose() {
        return Constants.kBlueReefBPose;
    }
    
    public TargetPose getBlueCoralA1Pose() {
        return Constants.kBlueCoralA1Pose;
    }

    public TargetPose getBlueCoralA2Pose() {
        return Constants.kBlueCoralA2Pose;
    }
    
    public TargetPose getRedReefAPose() {
        return Constants.kRedReefAPose;
    }

    public TargetPose getRedReefBPose() {
        return Constants.kRedReefBPose;
    }
        
    public TargetPose getRedReefKPose() {
        return Constants.kRedReefKPose;
    }

    public TargetPose getRedReefLPose() {
        return Constants.kRedReefLPose;
    }

    public TargetPose getRedCoralA1Pose() {
        return Constants.kRedCoralA1Pose;
    }

    public TargetPose getRedCoralA2Pose() {
        return Constants.kRedCoralA2Pose;
    }
}
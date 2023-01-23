// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class Limelight extends SubsystemBase {
  //Entries on camera server
  public NetworkTableEntry camMode = CameraConstants.camMode;
  public NetworkTableEntry pipeline = CameraConstants.pipeline;
  public NetworkTableEntry stream = CameraConstants.stream;
  public NetworkTable table = CameraConstants.table;

  //Data returned by AprilTag tracking
  private PhotonPipelineResult result;
  private boolean hasTargets;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget target;
  private double yaw;
  private double pitch;
  private double area;
  private double poseAmbiguity;
  private int targetID;


  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget target;

  private double yaw;
  private double pitch;
  private double area;
  private double poseAmbiguity;

  private int targetID;

  private Transform3d bestCameraToTarget;
  private Transform3d alternateCameraToTarget;

  private double rangeToTarget; 
  private double distanceToTarget;



  private Transform3d bestCameraToTarget;
  private Transform3d alternateCameraToTarget;
  private double rangeToTarget; 
  private double distanceToTarget;

  
  private Translation2d translation;

  private Pose3d robotPose;
  
  HttpCamera feed;
  // AprilTagFieldLayout aprilTagFieldLayout = new
  // AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource
  // (AprilTagFields.k2022RapidReact.m_resourceFile));
  private final HttpCamera feed;

  private final PhotonCamera m_camera;

  

  public Limelight() {
    feed = new HttpCamera("photonvision", "http://10.8.10.11:5800/");
    CameraServer.startAutomaticCapture(feed);

    m_camera = new PhotonCamera("photonvision");
  }

  public void shuffleUpdate()
  {
    result = m_camera.getLatestResult();
    target = result.getBestTarget();

  }

  public void aprilTagData() {
    result.hasTargets();

  }

  public void turnToTarget()
  {
    aprilTagData();
    if(result.hasTargets())
    {
      // yaw 
    }
  }

  public void aprilTagData() {
    this.hasTargets = result.hasTargets();


    targets = result.getTargets();

    target = result.getBestTarget();

    yaw = target.getYaw();
    pitch = target.getPitch();
    area = target.getArea();


    targetID = target.getFiducialId();
    poseAmbiguity = target.getPoseAmbiguity();
    bestCameraToTarget = target.getBestCameraToTarget();
    alternateCameraToTarget = target.getAlternateCameraToTarget();

    if(result.hasTargets()){
      PhotonUtils.calculateDistanceToTargetMeters(
              CameraConstants.CAMERA_HEIGHT_METERS,
              CameraConstants.TEST_TARGET_HEIGHT_METERS, 
              CameraConstants.CAMERA_PITCH_RADIANS, 
              Units.degreesToRadians(result.getBestTarget().getPitch()));
    }

    
    //Translation2d
    translation = PhotonUtils.estimateCameraToTargetTranslation(
                  Constants.TEST_TARGET_HEIGHT_METERS, 
                  Rotation2d.fromDegrees(-target.getYaw()));

  //robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
  //            target.getBestCameraToTarget(), 
  //            AprilTagFieldLayout.getTagPose(target.getFiducialId()), 
  //            alternateCameraToTarget);
  }


    // Transform3D

    // Pose3d  robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
    //                                 target.getBestCameraToTarget(), 
    //                                 AprilTagFieldLayout.getTagPose(target.getFiducialId()), 
    //                                 alternateCameraToTarget);
 }


  public void setMode(int mode)
  {
    switch(mode){
      case 0:
        //AprilTag long range

        if(result.hasTargets() != true)
        {
          pipeline.setInteger(0);
        }
        else
        {
          pipeline.setInteger(2);
        }
        break;
      case 1:
        //Reflective Tape //long range ?
        pipeline.setInteger(1);
        break;
      case 2:
        //Processing
        pipeline.setInteger(3);
        break;

        pipeline.setInteger(0);
        break;
      case 1:
        //Reflective Tape //long range ? 
        pipeline.setInteger(1);
        break;
      case 2:
        //AprilTag short range
        pipeline.setInteger(2);
        break;
      case 3:
        //Processing
        pipeline.setInteger(3);
        break;

    }
  }


  @Override
  public void periodic() {
    // shuffleUpdate();
  }
}

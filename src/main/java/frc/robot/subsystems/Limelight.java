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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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

  public Translation3d idk;
  private Rotation3d hey;
  private Transform3d bestCameraToTarget;
  private Transform3d alternateCameraToTarget;
  private double rangeToTarget; 
  private double distanceToTarget;

  
  private Translation2d translation;

  private Pose3d robotPose;
  
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
    Transform3D

    Pose3d  robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                                    target.getBestCameraToTarget(), 
                                    AprilTagFieldLayout.getTagPose(target.getFiducialId()), 
                                    alternateCameraToTarget);
 }

  public void setMode(int mode)
  {
    switch(mode){
      case 1:
        //AprilTags
        pipeline.setInteger(0);
        
        System.out.println("Vision mode: AprilTag, Current Mode: " + pipeline.getDouble(-1));
        break;
      case 2:
        //Reflective Tape
        pipeline.setInteger(1);
        System.out.println("Vision mode: Limelight, Current Mode: " + pipeline.getDouble(-1));
        break;
      case 3:
        //Processing
        pipeline.setInteger(2);
        break;
    }
  }

  @Override
  public void periodic() {
    // shuffleUpdate();
  }
}

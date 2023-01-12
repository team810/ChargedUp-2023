// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  public NetworkTableEntry camMode = Constants.camMode;

  public NetworkTableEntry pipeline = Constants.pipeline;
  public NetworkTableEntry stream = Constants.stream;

  private PhotonPipelineResult result;

  private boolean hasTargets;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget target;

  private double yaw;
  private double pitch;
  private double area;
  private double poseAmbiguity;

  private int targetID;

  private Transform3d bestCameraToTarget;
  private Transform3d alternateCameraToTarget;

  HttpCamera feed;
  // AprilTagFieldLayout aprilTagFieldLayout = new
  // AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

  /** Creates a new Limelight. */

  PhotonCamera m_camera;
  DrivetrainSubsystem m_drivetrain;
  CommandXboxController m_controller;

  public Limelight() {
    feed = new HttpCamera("photonvision", "http://10.8.10.11:5800/");
    CameraServer.startAutomaticCapture(feed);

    m_camera = new PhotonCamera("photonvision");

    setMode(2);
  }

  public void shuffleUpdate()
  {
    result = m_camera.getLatestResult();
    target = result.getBestTarget();
  }

  public void aprilTagData() {
    hasTargets = result.hasTargets();

    targets = result.getTargets();

    target = result.getBestTarget();

    yaw = target.getYaw();
    pitch = target.getPitch();
    area = target.getArea();

    targetID = target.getFiducialId();
    poseAmbiguity = target.getPoseAmbiguity();
    bestCameraToTarget = target.getBestCameraToTarget();
    alternateCameraToTarget = target.getAlternateCameraToTarget();
  }

  private void toggleLED(boolean onOff)
  {
    if(onOff)
      m_camera.setLED(VisionLEDMode.kOn);
    else
      m_camera.setLED(VisionLEDMode.kOff);
  }

  private void setMode(int mode)
  {
    switch(mode){
      case 1:
        pipeline.setNumber(1);
        camMode.setBoolean(false);
        // toggleLED(false);
        break;
      case 2:
        pipeline.setNumber(2);
        // toggleLED(true);
        camMode.setBoolean(false);
        break;
      case 3:
        camMode.setBoolean(true);
      break;
    }
  }

  @Override
  public void periodic() {
    // shuffleUpdate();
  }
}

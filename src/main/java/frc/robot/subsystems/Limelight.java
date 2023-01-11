// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  public NetworkTableEntry ledMode = Constants.ledMode;
  public NetworkTableEntry camMode = Constants.camMode;

  public NetworkTableEntry pipeline = Constants.pipeline;
  public NetworkTableEntry stream = Constants.stream;

  public PhotonPipelineResult result;
  public boolean hasTargets;

  public List<PhotonTrackedTarget> targets;

  public PhotonTrackedTarget target;

  public double yaw;
  public double pitch;
  public double area;

  public int targetID;
  public double poseAmbiguity;
  public Transform3d bestCameraToTarget;
  public Transform3d alternateCameraToTarget;

  HttpCamera feed;
  // AprilTagFieldLayout aprilTagFieldLayout = new
  // AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

  /** Creates a new Limelight. */

  PhotonCamera m_camera;
  DrivetrainSubsystem m_drivetrain;
  CommandXboxController m_controller;

  public Limelight() {
    feed = new HttpCamera("limelight", "http://photonvision.local:5800/");
    CameraServer.startAutomaticCapture(feed);

    m_camera = new PhotonCamera("photonvision");

    m_camera.setLED(VisionLEDMode.kOn);
  }

  public void shuffleUpdate()
  {
    result = m_camera.getLatestResult();
    target = result.getBestTarget();

    SmartDashboard.putNumber("Tag ID", target.getFiducialId());
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

  @Override
  public void periodic() {
    // shuffleUpdate();
  }
}

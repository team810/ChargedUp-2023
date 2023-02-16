// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class Limelight extends SubsystemBase {
  // Entries on camera server
  private final NetworkTableEntry pipeline = CameraConstants.pipeline;
  private final NetworkTableEntry targetPixelsX = CameraConstants.targetPixelsX;
  private final HttpCamera feed;
  private final PhotonCamera m_camera;

  private PhotonPipelineResult result;

  private final ShuffleboardLayout tab = CameraConstants.CAMERA_VALUES;

  public Limelight() {
    feed = new HttpCamera("photonvision", "http://10.8.10.11:5800/");
    CameraServer.startAutomaticCapture(feed);

    m_camera = new PhotonCamera("photonvision");

    setMode("AprilTag");

    shuffleInit();
  }

  public double getTargetPixelsX() {
    return this.targetPixelsX.getDouble(-1);
  }

  public PhotonTrackedTarget getBestTarget() {
    return m_camera.getLatestResult().getBestTarget();
  }

  public void setMode(String pipeline) {
    switch (pipeline) {
      case "AprilTag":
        // long range 0, short 2
        this.pipeline.setInteger(0);
        break;
      case "Reflective Tape":
        this.pipeline.setInteger(1);
        break;
    }
  }

  public void shuffleInit() {
    tab.getLayout("Limelight Values").addBoolean("Is Valid?", () -> result.hasTargets());
    tab.addCamera("Live View", "photonvision", "http://10.8.10.11:5800");
  }

  @Override
  public void periodic() {
    result = m_camera.getLatestResult();
  }
}

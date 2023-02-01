// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class Limelight extends SubsystemBase {
  // Entries on camera server
  public NetworkTableEntry camMode = CameraConstants.camMode;
  public NetworkTableEntry pipeline = CameraConstants.pipeline;
  public NetworkTableEntry stream = CameraConstants.stream;
  public NetworkTable table = CameraConstants.table;

  // Data returned by AprilTag tracking
  private PhotonPipelineResult result;
  private final HttpCamera feed;

  private final PhotonCamera m_camera;

  public Limelight() {
    feed = new HttpCamera("photonvision", "http://10.8.10.11:5800/");
    CameraServer.startAutomaticCapture(feed);

    m_camera = new PhotonCamera("photonvision");
  }

  public void shuffleUpdate() {
    result = m_camera.getLatestResult();
  }

  public void setMode(int mode) {
    switch (mode) {
      case 0:
        // AprilTag long range
        pipeline.setInteger(0);

        if (result.hasTargets()) {
          pipeline.setInteger(2);
        }

        break;
      case 1:
        // Reflective Tape
        pipeline.setInteger(1);
        break;
      case 2:
        // Processing
        pipeline.setInteger(3);
        break;

    }
  }

  @Override
  public void periodic() {
    shuffleUpdate();
  }
}

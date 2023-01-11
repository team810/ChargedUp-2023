// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  public NetworkTableEntry ledMode = Constants.ledMode;
  public NetworkTableEntry camMode = Constants.camMode;

  public NetworkTableEntry pipeline = Constants.pipeline;
  public NetworkTableEntry stream = Constants.stream;

  HttpCamera feed;  
  // AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

  /** Creates a new Limelight. */
  public Limelight() {
    feed = new HttpCamera("limelight", "http://photonvision.local:5800/");
    CameraServer.startAutomaticCapture(feed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

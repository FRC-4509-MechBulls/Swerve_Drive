// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//meaga cool super epic coding time I love code so much UwU -Isaac

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("gloworm");

  /** Creates a new Vision. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for(PhotonTrackedTarget target :camera.getLatestResult().getTargets()){
      int id = target.getFiducialId();
      Transform3d transform = target.getBestCameraToTarget();

      SmartDashboard.putNumber("tag"+id+".X", transform.getX());
      SmartDashboard.putNumber("tag"+id+".Y", transform.getY());
      SmartDashboard.putNumber("tag"+id+".Z", transform.getZ());

    }
  }
}

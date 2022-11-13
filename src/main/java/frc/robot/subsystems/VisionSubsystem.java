// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

      if(id==0){
        lastSeenOnRight = transform.getY()>0;
        lastTransform = transform;
        lastSeenTime = Timer.getFPGATimestamp();

      }

    }
  }



  boolean lastSeenOnRight;
  Transform3d lastTransform = new Transform3d();
  double lastSeenTime;
public double[] getDesiredSpeeds(){
  double[] out = new double[3];
  double timeSinceSeen = Timer.getFPGATimestamp() - lastSeenTime;

  if(timeSinceSeen>0.5){
    out[2] = -0.1;
    if(lastSeenOnRight)
      out[2] = 0.1;
    return out;
  }

  out[0] = lastTransform.getX()-1; // get one meter from target
  out[1] = lastTransform.getY();

  //out[2] = lastTransform.getRotation().getX(); //this might be the wrong axis, uncomment this for rotation tracking

  out[0] = Constants.absMax(out[0], 0.1);
  out[1] = Constants.absMax(out[1], 0.1);
  out[2] = Constants.absMax(out[2], 0.1);


  return out;
}

}

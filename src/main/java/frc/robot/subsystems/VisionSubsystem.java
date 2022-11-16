// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import frc.robot.RobotContainer;
import frc.robot.lib.FieldTag;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//meaga cool super epic coding time I love code so much UwU -Isaac

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("gloworm");
  /** Creates a new Vision. */

  private FieldTag[] fieldTags = new FieldTag[1];
  private SwerveSubsystem swerveSubsystem;
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    fieldTags[0] = new FieldTag(0, new Pose2d(-1, 0, new Rotation2d(Math.PI)));
  }




  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    for(PhotonTrackedTarget target :camera.getLatestResult().getTargets()){
      int id = target.getFiducialId();
      Transform3d transform = target.getBestCameraToTarget();
      Rotation3d rotation = transform.getRotation();

      for(int i = 0; i<fieldTags.length; i++){
        if(fieldTags[i].getID() != id) continue;
        Transform3d sentTransform = new Transform3d(new Translation3d(transform.getX()*-1,transform.getY()*-1,transform.getZ()),transform.getRotation());
        swerveSubsystem.fieldTagSpotted(fieldTags[i], transform);

      }

//      SmartDashboard.putNumber("tag"+id+".X", transform.getX());
//      SmartDashboard.putNumber("tag"+id+".Y", transform.getY());
//      SmartDashboard.putNumber("tag"+id+".Z", transform.getZ());
//
//      SmartDashboard.putNumber("tag"+id+".rX", rotation.getX());
//      SmartDashboard.putNumber("tag"+id+".rY", rotation.getY());
//      SmartDashboard.putNumber("tag"+id+".rZ", rotation.getZ());

      
  double rotationFromCamera = Math.IEEEremainder(lastTransform.getRotation().getZ() + Math.PI, 2*Math.PI);
  SmartDashboard.putNumber("rotationFromCamera", rotationFromCamera);


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

  out[0] = (lastTransform.getX()-1) *0.5; // get one meter from target
  out[1] = lastTransform.getY() * 0.5;
  SmartDashboard.putNumber("out0",out[0]);

  double rotationFromCamera = Math.IEEEremainder(lastTransform.getRotation().getZ() + Math.PI, 2*Math.PI);
  out[2] =  rotationFromCamera;//this might be the wrong axis, uncomment this for rotation tracking

  out[0] = Constants.absMax(out[0], 0.2);
  out[1] = Constants.absMax(out[1], 0.2);
  if(Math.abs(out[2])<0.1) out[2] = 0;
  out[2] = Constants.absMax(out[2], 0.05);


  return out;
}

}

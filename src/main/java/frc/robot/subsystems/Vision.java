// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 * 
 * 
 



 IP Adress: 10.76.17.11
 */
package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private PhotonCamera cyclops;
  private static Vision instance;
  private PhotonPipelineResult result;
  private boolean hasTargets;
  private double angle;
  private double distance;
  /** Creates a new Vision. */
  private Vision() {
    cyclops  =  new PhotonCamera("cyclops");
  }

  public static synchronized Vision getInstance(){
    if (instance != null)
      return instance;
    else  
      instance = new Vision();
      return instance;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = cyclops.getLatestResult();
    hasTargets = result.hasTargets();
    if(hasTargets){
      angle = -result.getBestTarget().getYaw();
      distance = PhotonUtils.calculateDistanceToTargetMeters(Constants.Vision.CAMERA_HEIGHT_METERS, Constants.Vision.TARGET_HEIGHT_METERS, Constants.Vision.CAMERA_PITCH_RADIANS, Constants.Vision.TARGET_PITCH_RADIANS);
    }
    else{
      angle = 0;
      distance = 0;
    }
  }

  public double getAngle(){
    return angle;
  }

  public double getDistance(){
    return distance;
  }

  public boolean hasTargets(){
    return hasTargets;
  }
}

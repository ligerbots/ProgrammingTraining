// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final PhotonCamera m_aprilTagCamera = new PhotonCamera("ApriltagCamera");
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void findDistanceToTag () {
    if (!m_aprilTagCamera.isConnected())
      return;
    var targetResult = m_aprilTagCamera.getLatestResult();
    PhotonTrackedTarget target = targetResult.getBestTarget();
    SmartDashboard.putNumber("vision/targetID", target.getFiducialId());
    Transform3d cameraToTarget = target.getBestCameraToTarget();
    SmartDashboard.putNumber("vision/tagOffsetX", Units.metersToInches(cameraToTarget.getX()));
    SmartDashboard.putNumber("vision/tagOffsetY", Units.metersToInches(cameraToTarget.getY()));
    SmartDashboard.putNumber("vision/tagOffsetYaw", Math.toDegrees(cameraToTarget.getRotation().getZ()));
  }

}

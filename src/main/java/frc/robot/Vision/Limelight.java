// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;
import frc.robot.Vision.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  private String ll = "limelight";
  private Boolean enable = true;
  private Boolean trust = false;
  private double confidence = 0;
  private double compareDistance;
  private Pose2d limelightPoseEstimate;

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
  private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("llPose").publish();
  private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0, 0), new Translation2d(16.541, 8.211));

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain, String ll) {
    this.drivetrain = drivetrain;
    this.ll = ll;
    LimelightHelpers.setPipelineIndex(ll, 0);
  }

  @Override
  public void periodic() {
    if ((enable || DriverStation.isDisabled()) && !RobotBase.isSimulation()) {
      // How to test:
      // Odometry is good on a nice flat surface so when testing on flat assume odometry as ground truth
      // Log over the last 5? seconds what has been the avg distance between odometry and the LL pose


      // Things to consider testing / excluding:
      // When spining past some rate we get bad results
      // Anything past 15ft seems to have too much variance
      // 

      confidence = 0; // If we don't update confidence then we don't send the measurement
      LimelightHelpers.SetRobotOrientation(ll, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
      LimelightHelpers.PoseEstimate limelightMeasurementNew = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
      SmartDashboard.putNumber("NumTags", limelightMeasurement.tagCount);

      // No tag found so check no further or pose not within field boundary
      if(limelightMeasurement.tagCount > 0 && fieldBoundary.isPoseWithinArea(limelightMeasurement.pose)) {
        // Excluding different measurements that are absolute showstoppers even with full trust 
        if(limelightMeasurement.avgTagDist < Units.feetToMeters(12)) {
          confidence = 0.5;
        } else {
          // If more than 12 ft away use MegaTag 2 use MT if less than 12
          limelightMeasurement = limelightMeasurementNew;
          confidence = 0.7;
        }
      }
      addPose(limelightMeasurement, confidence);
      limelightPoseEstimate = limelightMeasurement.pose;
    }
  }

  private void addPose(LimelightHelpers.PoseEstimate limelightMeasurement, double confide) {
    if(confide > 0) {
      // We are publishing this to view as a ghost to try and help determine when not to use the LL measurements
      publishToField(limelightMeasurement);
      SmartDashboard.putBoolean("PoseUpdate", true);
      SmartDashboard.putNumber("LLConfidence", confide);
      drivetrain.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds,
          VecBuilder.fill(confide, confide, 9999999));
    } else {
      SmartDashboard.putBoolean("PoseUpdate", false);
      // We are publishing this to view as a ghost to try and help determine when not to use the LL measurements
      publishToField(new LimelightHelpers.PoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0, new RawFiducial[0]));
    }
  }

  public Pose2d getPoseEstimate() {
    if (confidence == 0) {
      return null;
    } else {
      return limelightPoseEstimate;
    }
  }

  private void publishToField(LimelightHelpers.PoseEstimate limelightMeasurement) {
    // If you have a Field2D you can easily push it that way here.
    limelightPub.set(new double[] {
      limelightMeasurement.pose.getX(),
      limelightMeasurement.pose.getY(),
      limelightMeasurement.pose.getRotation().getDegrees()
    });
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }

  public Command blinkLEDS() {
    return runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(ll))
        .andThen(waitSeconds(2))
        .andThen(() -> LimelightHelpers.setLEDMode_ForceOff(ll));
  }

  public Command ledsOff() {
    return runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(ll));
  }
}
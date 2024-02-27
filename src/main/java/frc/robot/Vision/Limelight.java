// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  private String ll = "limelight";
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private double confidence = 0;
  private double compareDistance;

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
  private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("llPose").publish();

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
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
      SmartDashboard.putNumber("NumTags", limelightMeasurement.tagCount);
      // We are publishing this to view as a ghost to try and help determine when not to use the LL measurements
      publishToField(limelightMeasurement);

      // No tag found so check no further
      if(limelightMeasurement.tagCount >= 1) {
        // Excluding different measurements that are absolute showstoppers even with full trust 
        if(limelightMeasurement.avgTagDist < Units.feetToMeters(15) && drivetrain.getState().speeds.omegaRadiansPerSecond < Math.PI) {
          // Reasons to blindly trust as much as odometry
          if (trust || DriverStation.isDisabled() || 
              (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagDist < Units.feetToMeters(10))) {
                confidence = 0.1;
          } else {
            compareDistance = limelightMeasurement.pose.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());
            if( compareDistance < 0.5) {

            }
          }
        }
      }
      addPose(limelightMeasurement, confidence);
    }
  }

  private void addPose(LimelightHelpers.PoseEstimate limelightMeasurement, double confide) {
    if(confide > 0) {
      SmartDashboard.putBoolean("PoseUpdate", true);
      drivetrain.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds,
          VecBuilder.fill(confide, confide, Double.MAX_VALUE));
    } else {
      SmartDashboard.putBoolean("PoseUpdate", false);
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
}

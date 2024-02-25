// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = "limelight-tag";
  private Boolean enable = true;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  Double targetDistance;
  private Pose2d botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
  private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("llPose").publish();

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain, String ll) {
    this.drivetrain = drivetrain;
    this.ll = ll;
    LimelightHelpers.setPipelineIndex(ll, 0);
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }

  @Override
  public void periodic() {
    if (enable && !RobotBase.isSimulation()) {
/*       // Get the distance between the camera and the AprilTag, this will affect how much we trust the measurement
      targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());
      // Tune this for your robot around how much variance you see in the pose at a given distance, higher = less trust
      Double lackconfidence = ((targetDistance - 1) / 6);
      LimelightHelpers.Results result =
          LimelightHelpers.getLatestResults(ll).targetingResults;
      if (result.valid) {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(ll);
        limelightPub.set(new double[] {
          botpose.getX(),
          botpose.getY(),
          botpose.getRotation().getDegrees()
        });
        if (field.isPoseWithinArea(botpose)) {
          int numberOfTargets = result.targets_Fiducials.length;
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
              || trust
              || numberOfTargets > 1) {
            drivetrain.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                VecBuilder.fill(lackconfidence / numberOfTargets, lackconfidence / numberOfTargets, .99));
          } else {
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
          }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      } */

      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
      SmartDashboard.putNumber("NumTags", limelightMeasurement.tagCount);
      if(limelightMeasurement.tagCount >= 2)
      {
        limelightPub.set(new double[] {
          limelightMeasurement.pose.getX(),
          limelightMeasurement.pose.getY(),
          limelightMeasurement.pose.getRotation().getDegrees()
        });
        drivetrain.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds,
            VecBuilder.fill(.7,.7,99));
      }
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }
}

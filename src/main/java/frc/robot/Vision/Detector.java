package frc.robot.Vision;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj2.command.Command;

public class Detector {
  private String ll;
  public Detector (String ll) {
    this.ll = ll;
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(ll);
  }

  public double getNoteHorizontal() {
    return LimelightHelpers.getTX(ll);
  }

  public double getNoteVertical() {
    return LimelightHelpers.getTY(ll);
  }

  public void setPipeline(Integer pipeline) {
    LimelightHelpers.setPipelineIndex(ll, pipeline);
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

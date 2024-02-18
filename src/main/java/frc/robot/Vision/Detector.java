package frc.robot.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Detector {
  private String ll;
  public Detector (String ll) {
    this.ll = ll;
    SmartDashboard.putString("llname", ll);
    LimelightHelpers.setPipelineIndex(ll, 1);
  }

  public void setPipeline() {
    SmartDashboard.putString("pipe", "1");
    LimelightHelpers.setPipelineIndex(ll, 1);
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(ll);
  }

  public double getNoteHorizontal() {
    return LimelightHelpers.getTX(ll);
  }
}

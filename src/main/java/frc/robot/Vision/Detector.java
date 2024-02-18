package frc.robot.Vision;

public class Detector {
  private String ll;
  public Detector (String ll) {
    this.ll = ll;
    LimelightHelpers.setPipelineIndex(ll, 1);
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(ll);
  }

  public double getNoteHorizontal() {
    return LimelightHelpers.getTX(ll);
  }
}

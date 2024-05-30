package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class Drive {
    public static final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
    public static final double MaxAngularRate = Math.PI * 1.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    public static final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
  }

  public static final class Field {
    public static final Translation2d blueSpeaker = new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(223.42)); // added 5"
    public static final Translation2d redSpeaker = new Translation2d(Units.inchesToMeters(642.73), Units.inchesToMeters(213.42)); // subtracted 5"

    public static final Translation2d blueCorner = new Translation2d(0.0, 8.211);
    public static final Translation2d redCorner = new Translation2d(16.541, 8.211);

    public static final double noteVelocity = 20.0; // We can tune this to get our shots hitting correctly empirically
  }
}

package frc.robot.Util;

// Shot parameter
public class ShotParameter {

    // Variables
    public final double angle;
    public final double rps;

    // Constructor
    public ShotParameter(double angle, double rps) {
        this.angle = angle;
        this.rps = rps;
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.angle - other.angle) < 0.1 &&
        Math.abs(this.rps - other.rps) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(angle, end.angle, t), 
            lerp(rps, end.rps, t)
        );
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
 
}
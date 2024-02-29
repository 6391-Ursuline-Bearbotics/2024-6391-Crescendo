package frc.robot.Util;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.util.Units;

import static java.util.Map.entry;

// Interpolating table
public class InterpolatingTable {
    public static final ShotParameter sub = new ShotParameter(8.0, 100); // 37.991 along ground
    public static final ShotParameter auto = new ShotParameter(28.0, 125); // 120.732 along ground
    public static final ShotParameter stage = new ShotParameter(41.5, 200); // 229.263 along ground
    public static final ShotParameter wing = new ShotParameter(47.6, 220); // 229.263 along ground

    /* Private constructor because this is a utility class */
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries( // using pythag with 33" vertical distance between camera and speaker tag
            entry(1.135, sub),
            entry(2.18, auto),
            entry(3.57, stage),
            entry(6.17, wing)
        )
    );

    // Method to get shot parameters based on vision distances
    public static ShotParameter get(double distance) {
        Entry<Double, ShotParameter> ceilEntry = map.ceilingEntry(distance);
        Entry<Double, ShotParameter> floorEntry = map.floorEntry(distance);
        if (ceilEntry == null) return floorEntry.getValue();
        if (floorEntry == null) return ceilEntry.getValue();
        if (ceilEntry.getValue().equals(floorEntry.getValue())) return ceilEntry.getValue();
        return ceilEntry.getValue().interpolate(
            floorEntry.getValue(), 
            (distance - floorEntry.getKey())/(ceilEntry.getKey() - floorEntry.getKey())
        );
    }

}

package frc.robot.Util;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.util.Units;

import static java.util.Map.entry;

// Interpolating table
public class InterpolatingTable {

    /* Private constructor because this is a utility class */
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries( // using pythag with 33" vertical distance between camera and speaker tag
            entry(Units.inchesToMeters(50.32), new ShotParameter(7.0, 100)), // 37.991 along ground
            entry(Units.inchesToMeters(125.16), new ShotParameter(22.0, 100)), // 120.732 along ground
            entry(Units.inchesToMeters(231.63), new ShotParameter(40.0, 300)) // 229.263 along ground
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

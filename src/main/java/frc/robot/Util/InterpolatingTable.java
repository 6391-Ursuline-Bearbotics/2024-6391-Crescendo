package frc.robot.Util;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.util.Units;

import static java.util.Map.entry;

// Interpolating table
public class InterpolatingTable {
    public static final ShotParameter sub = new ShotParameter(10.0, 125); //8.0 100
    public static final ShotParameter auto = new ShotParameter(31.0, 140); //27
    public static final ShotParameter stage = new ShotParameter(43.5, 200); //42
    public static final ShotParameter wing = new ShotParameter(49.2, 220); // 48.2
    public static final ShotParameter farwing = new ShotParameter(38.3, 142); //36.3

    /* Private constructor because this is a utility class */
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            entry(1.20, sub),
            entry(2.21, auto),
            entry(3.3, stage),
            entry(6.24, wing),
            entry(10.18, farwing)
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

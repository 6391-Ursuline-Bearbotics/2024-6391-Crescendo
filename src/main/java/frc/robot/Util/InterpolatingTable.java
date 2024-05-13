package frc.robot.Util;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.util.Units;

import static java.util.Map.entry;

// Interpolating table
public class InterpolatingTable {
    public static final ShotParameter sub = new ShotParameter(19, 115); //11 125
    public static final ShotParameter auto = new ShotParameter(30.5, 130); //31.2 140
    public static final ShotParameter stage = new ShotParameter(38, 150); //42.5 200
    public static final ShotParameter wing = new ShotParameter(48, 180); // 53 220
    public static final ShotParameter farwing = new ShotParameter(38.3, 142); //38.3 142

    /* Private constructor because this is a utility class */
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            entry(1.19, sub), // 1.15
            entry(2.12, auto), // 2.08
            entry(3.36, stage), // 3.39
            entry(6.20, wing), // 6.2
            entry(10.18, farwing) // 10.18
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

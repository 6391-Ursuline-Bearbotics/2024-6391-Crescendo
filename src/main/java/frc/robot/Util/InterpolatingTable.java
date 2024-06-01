package frc.robot.Util;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.util.Units;

import static java.util.Map.entry;

// Interpolating table
public class InterpolatingTable {
    public static final ShotParameter sub = new ShotParameter(18.5, 115); //11 125
    public static final ShotParameter auto = new ShotParameter(30.5, 130); //31.2 140
    public static final ShotParameter stage = new ShotParameter(38, 150); //42.5 200
    public static final ShotParameter wing = new ShotParameter(48, 160); // 48 180
    public static final ShotParameter fwing = new ShotParameter(30, 100); // 32 115
    public static final ShotParameter farwing = new ShotParameter(30, 100); //32 115

    /* Private constructor because this is a utility class */
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            entry(1.19, sub), // 1.19 if sub shot seems off then adjust to 1.16
            entry(2.12, auto), // 2.12
            entry(3.36, stage), // 3.36
            entry(6.20, wing), // 6.2
            entry(6.80, fwing), // 7.2
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

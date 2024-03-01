package frc.robot.Util;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.util.Units;

import static java.util.Map.entry;

// Interpolating table
public class NoteInterpolation {
    /* Private constructor because this is a utility class */
    private NoteInterpolation() {}

    // Interpolating tree map
    private static final TreeMap<Double, Double> yMap = new TreeMap<>(
        Map.ofEntries(
            entry(1.135, 0.0),
            entry(2.18, 0.0),
            entry(3.61, 0.0),
            entry(6.17, 0.0)
        )
    );

/*     // Interpolating tree map
    private static final TreeMap<Double, Double> xMap = new TreeMap<>(
        Map.ofEntries(
            entry(1.135, sub),
            entry(2.18, auto),
            entry(3.61, stage),
            entry(6.17, wing)
        )
    );

    // Method to get a pose based on vision angles
    public static Double get(double angle) {
        Entry<Double, Double> ceilEntry = xMap.ceilingEntry(angle);
        Entry<Double, Double> floorEntry = xMap.floorEntry(angle);
        if (ceilEntry == null) return floorEntry.getValue();
        if (floorEntry == null) return ceilEntry.getValue();
        if (ceilEntry.getValue().equals(floorEntry.getValue())) return ceilEntry.getValue();
        return ceilEntry.getValue().interpolate(
            floorEntry.getValue(), 
            (angle - floorEntry.getKey())/(ceilEntry.getKey() - floorEntry.getKey())
        );
    } */

}

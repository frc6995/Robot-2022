package frc.robot.util.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;

// Courtesy of 5940

// Interpolating table
public class ShooterInterpolatingTable {

    /* Private constructor because this is a utility class */
    private ShooterInterpolatingTable() {}

    public static final double MIN_DISTANCE = 1.5;
    public static final double MAX_DISTANCE = 8.5;

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            entry(1.5, new ShotParameter(2750, 2750, 1.0)),
            entry(2.5, new ShotParameter(3000, 3000, 1.0)),
            entry(3.5, new ShotParameter(3000, 3000, 1.0)),
            entry(4.5, new ShotParameter(3000, 3000, 1.0)),
            entry(5.5, new ShotParameter(3200, 3200, 1.0)),
            entry(6.5, new ShotParameter(3250, 3250, 1.0)),
            entry(7.5, new ShotParameter(3400, 3400, 1.0)),
            entry(8.5, new ShotParameter(3850, 3850, 1.0))
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
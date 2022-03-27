package frc.robot.util.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import frc.robot.Constants;

import static java.util.Map.entry;

// Courtesy of 5940

// Interpolating table
public class ShooterInterpolatingTable {

  /* Private constructor because this is a utility class */
  private ShooterInterpolatingTable() {
  }

  public static final double MIN_DISTANCE = 2.5;
  public static final double MAX_DISTANCE = 4.3;

  // Interpolating tree map
  private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(Map.ofEntries(
      // entry(2.3 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(1550, 1550,
      // 1.0)),
      entry(2.5 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(1550, 1550, 1.0)),
      entry(2.65 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(1550, 1550, 1.0)),
      entry(2.9 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(1600, 1600, 1.0)),
      entry(3.52 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(1900, 1650, 1.0))));
  // entry(5.4 + Constants.CAMERA_CENTER_OFFSET, new ShotParameter(1900, 1900,
  // 1.0))));

  // Method to get shot parameters based on vision distances
  public static ShotParameter get(double distance) {
    Entry<Double, ShotParameter> ceilEntry = map.ceilingEntry(distance);
    Entry<Double, ShotParameter> floorEntry = map.floorEntry(distance);
    if (ceilEntry == null)
      return floorEntry.getValue();
    if (floorEntry == null)
      return ceilEntry.getValue();
    if (ceilEntry.getValue().equals(floorEntry.getValue()))
      return ceilEntry.getValue();
    return ceilEntry.getValue().interpolate(
        floorEntry.getValue(),
        (distance - floorEntry.getKey()) / (ceilEntry.getKey() - floorEntry.getKey()));
  }

}
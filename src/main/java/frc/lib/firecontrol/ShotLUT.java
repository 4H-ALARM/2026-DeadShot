/*
 * ShotLUT.java - Adjustable-hood shot lookup container
 *
 * MIT License
 */
package frc.lib.firecontrol;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/** Bundled LUT for distance -> RPM, hood angle, and TOF. */
public class ShotLUT {
  private final NavigableMap<Double, ShotParameters> entries = new TreeMap<>();

  public void put(double distanceM, double rpm, double hoodAngleDeg, double tofSec) {
    entries.put(distanceM, new ShotParameters(distanceM, rpm, hoodAngleDeg, tofSec));
  }

  public Collection<ShotParameters> entries() {
    return Collections.unmodifiableCollection(new ArrayList<>(entries.values()));
  }

  public int size() {
    return entries.size();
  }

  public ShotParameters getClosest(double distanceM) {
    Map.Entry<Double, ShotParameters> floor = entries.floorEntry(distanceM);
    Map.Entry<Double, ShotParameters> ceil = entries.ceilingEntry(distanceM);
    if (floor == null) {
      return ceil != null ? ceil.getValue() : null;
    }
    if (ceil == null) {
      return floor.getValue();
    }
    return Math.abs(distanceM - floor.getKey()) <= Math.abs(ceil.getKey() - distanceM)
        ? floor.getValue()
        : ceil.getValue();
  }
}

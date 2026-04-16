/*
 * ShotParameters.java - Distance-indexed shot parameters
 *
 * MIT License
 */
package frc.lib.firecontrol;

/** One shot solution row for an adjustable-hood LUT. */
public record ShotParameters(double distanceM, double rpm, double hoodAngleDeg, double tofSec) {}

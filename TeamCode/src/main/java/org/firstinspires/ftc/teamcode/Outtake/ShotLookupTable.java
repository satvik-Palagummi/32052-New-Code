package org.firstinspires.ftc.teamcode.Outtake;

import java.util.TreeMap;
import java.util.Map;

/**
 * A utility to find the correct shooter power and hood angle based on distance.
 * Values are based on calibration data from Frankenstein.java.
 */
public class ShotLookupTable {

    public static class ShotParams {
        public double flywheelVelocity; // Ticks per second
        public double hoodPosition;     // Axon Max Servo position (0.0 to 1.0)

        public ShotParams(double flywheelVelocity, double hoodPosition) {
            this.flywheelVelocity = flywheelVelocity;
            this.hoodPosition = hoodPosition;
        }
    }

    private static final TreeMap<Double, ShotParams> table = new TreeMap<>();

    static {
        // Distance (cm) -> {Flywheel Velocity (ticks/sec), Hood Position (0.0 to 1.0)}
        // These values match the distance ranges found in the original Frankenstein.java
        
        // Close range (distance < 160)
        table.put(0.0,   new ShotParams(1370, 0.1));
        table.put(159.0, new ShotParams(1370, 0.2));
        
        // Mid-Close range (160 < distance < 180)
        table.put(170.0, new ShotParams(1410, 0.3));
        
        // Mid range (180 < distance < 220)
        table.put(200.0, new ShotParams(1460, 0.45));
        
        // Far range (320 < distance < 335)
        table.put(327.0, new ShotParams(1720, 0.75));
        
        // Extreme range (distance > 335)
        table.put(350.0, new ShotParams(1740, 0.85));
        
        // Safety ceiling
        table.put(1000.0, new ShotParams(1740, 0.9));
    }

    /**
     * Gets interpolated shot parameters based on distance using linear interpolation.
     * @param distance Distance to the target in cm (from Limelight).
     * @return ShotParams containing flywheel velocity and hood servo position.
     */
    public static ShotParams getParams(double distance) {
        // Find the data points immediately below and above the current distance
        Map.Entry<Double, ShotParams> low = table.floorEntry(distance);
        Map.Entry<Double, ShotParams> high = table.ceilingEntry(distance);

        // Handle edge cases where distance is outside table bounds
        if (low == null && high == null) return new ShotParams(1370, 0.1);
        if (low == null) return high.getValue();
        if (high == null) return low.getValue();
        if (low.getKey().equals(high.getKey())) return low.getValue();

        // Linear interpolation formula:
        // 1. Find where the distance sits between the two points (0.0 to 1.0)
        double t = (distance - low.getKey()) / (high.getKey() - low.getKey());

        // 2. Apply that ratio to both Velocity and Hood Position
        double vel = low.getValue().flywheelVelocity + t * (high.getValue().flywheelVelocity - low.getValue().flywheelVelocity);
        double hood = low.getValue().hoodPosition + t * (high.getValue().hoodPosition - low.getValue().hoodPosition);

        return new ShotParams(vel, hood);
    }
}

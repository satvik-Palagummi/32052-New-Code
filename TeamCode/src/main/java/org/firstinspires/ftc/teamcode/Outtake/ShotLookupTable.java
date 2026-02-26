package org.firstinspires.ftc.teamcode.Outtake;

import java.util.TreeMap;
import java.util.Map;

public class ShotLookupTable {
    
    public static class ShotParams {
        public double flywheelVelocity;
        public int hoodPosition;

        public ShotParams(double flywheelVelocity, int hoodPosition) {
            this.flywheelVelocity = flywheelVelocity;
            this.hoodPosition = hoodPosition;
        }
    }

    private static final TreeMap<Double, ShotParams> table = new TreeMap<>();

    static {
        // Distance (cm) -> {Flywheel Velocity (ticks/sec), Hood Position (ticks)}
        // These are example values based on  existing code TODO should be tuned.
        table.put(0.0, new ShotParams(1370, 0));
        table.put(150.0, new ShotParams(1370, 100));
        table.put(170.0, new ShotParams(1410, 200));
        table.put(200.0, new ShotParams(1460, 350));
        table.put(320.0, new ShotParams(1720, 800));
        table.put(340.0, new ShotParams(1740, 950));
        table.put(1000.0, new ShotParams(1740, 1000));
    }

    /**
     * Gets interpolated shot parameters based on distance.
     * @param distance Distance to the target in cm.
     * @return ShotParams containing flywheel velocity and hood position.
     */
    public static ShotParams getParams(double distance) {
        Map.Entry<Double, ShotParams> low = table.floorEntry(distance);
        Map.Entry<Double, ShotParams> high = table.ceilingEntry(distance);

        if (low == null && high == null) return new ShotParams(1370, 0);
        if (low == null) return high.getValue();
        if (high == null) return low.getValue();
        if (low.getKey().equals(high.getKey())) return low.getValue();

        double t = (distance - low.getKey()) / (high.getKey() - low.getKey());
        
        double vel = low.getValue().flywheelVelocity + t * (high.getValue().flywheelVelocity - low.getValue().flywheelVelocity);
        int hood = (int) (low.getValue().hoodPosition + t * (high.getValue().hoodPosition - low.getValue().hoodPosition));

        return new ShotParams(vel, hood);
    }
}

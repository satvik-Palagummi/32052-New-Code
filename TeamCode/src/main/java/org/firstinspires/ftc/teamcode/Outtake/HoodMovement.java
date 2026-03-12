package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Handles the vertical angle of the shooter hood using an Axon Max Servo.
 */
public class HoodMovement {
    private Servo hood;

    /**
     * Initializes the hood servo.
     * Matches the name "hood" used in the original hardware configuration.
     */
    public void initHood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "hood");
    }

    /**
     * Moves the hood to a specific servo position.
     * @param position The target servo position (0.0 to 1.0).
     */
    public void setHoodPosition(double position){
        if (hood != null) {
            hood.setPosition(position);
        }
    }

    /**
     * Returns the current target position of the hood servo.
     */
    public double getHoodPosition() {
        return (hood != null) ? hood.getPosition() : 0.0;
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FrankensteinRed extends Frankenstein {

    @Override
    public int getLimelightPipeline() {
        return 9;
    }

    @Override
    public double toTheLeftFar() {
        return -4;
    }

    @Override
    public double toTheRightFar() {
        return 2;
    }
}

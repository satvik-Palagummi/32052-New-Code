package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FrankensteinBlue extends Frankenstein{
    @Override
    public int getLimelightPipeline() {
        return 8;
    }

    @Override
    public double toTheLeftFar() {
        return -2;
    }

    @Override
    public double toTheRightFar() {
        return 4;
    }
}

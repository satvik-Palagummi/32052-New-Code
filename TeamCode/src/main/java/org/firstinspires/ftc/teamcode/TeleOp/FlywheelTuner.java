package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class FlywheelTuner extends OpMode {
    public DcMotorEx turretL;
    public DcMotorEx turretR;
    double highVelocity = 1640;
    double curTargetVelocity = highVelocity;
    double lowVelocity = 900;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex  = 1;

    @Override
    public void init() {
        turretL = hardwareMap.get(DcMotorEx.class, "TurretL");
        turretR = hardwareMap.get(DcMotorEx.class, "TurretR");
        turretL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretL.setDirection(DcMotorEx.Direction.FORWARD);
        turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretR.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        turretL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        turretR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");

    }

    @Override
    public void loop() {
        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            }else{
                curTargetVelocity = highVelocity;
            }
        }
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadLeftWasPressed()){
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        turretL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        turretR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        turretL.setVelocity(curTargetVelocity);
        double curLVelocity = turretL.getVelocity();
        double curRVelocity = turretR.getVelocity();
        double errorL = curTargetVelocity - curLVelocity;
        double errorR = curTargetVelocity - curRVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Left Velocity", curLVelocity);
        telemetry.addData("Current Right Velocity", curRVelocity);
        telemetry.addData("Error Left", errorL);
        telemetry.addData("Error Right", errorR);
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Tuning P", P);
        telemetry.addData("Tuning F",  F);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
    }
}

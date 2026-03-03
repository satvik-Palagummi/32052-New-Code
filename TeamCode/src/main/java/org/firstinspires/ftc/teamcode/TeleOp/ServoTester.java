package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Outtake.HoodMovement;
import org.firstinspires.ftc.teamcode.Outtake.PushServo;

@TeleOp
public class ServoTester extends LinearOpMode {
    PushServo pushServo = new PushServo();
    HoodMovement hoodMovement = new HoodMovement();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            handleServos();
            telemetry.addData("Hood Position", hoodMovement.hoodPos());
        }
    }

    public void initialize(){
        pushServo.initPushServos(hardwareMap);
        hoodMovement.initHoodServo(hardwareMap);
    }
    public void handleServos(){
        if(gamepad1.y){
            pushServo.propel(0);
        }else{
            pushServo.retract(0);
        }
        if(gamepad1.b){
            pushServo.propel(1);
        }else{
            pushServo.retract(1);
        }
        if(gamepad1.a){
            pushServo.propel(2);
        }else{
            pushServo.retract(2);
        }
        if(gamepad1.xWasPressed()){
            hoodMovement.setHoodZero();
        }
        if(gamepad1.dpadUpWasPressed()){
            hoodMovement.adjustHoodUp();
        }
        if(gamepad1.dpadDownWasPressed()){
            hoodMovement.adjustHoodDown();
        }
        if(gamepad1.rightBumperWasPressed()){
            hoodMovement.setHoodOne();
        }
    }
}

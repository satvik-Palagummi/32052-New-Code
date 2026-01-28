package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class TestWheels extends LinearOpMode {
    private DcMotorSimple frontLeftMotor;
    private DcMotorSimple backLeftMotor;
    private DcMotorSimple backRightMotor;
    private DcMotorSimple frontRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            handleDriving();
        }
    }

    public void initialize(){
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void handleDriving(){
        if(gamepad1.y){
            frontLeftMotor.setPower(1);
        }else{
            frontLeftMotor.setPower(0);
        }
        if(gamepad1.b){
            frontRightMotor.setPower(1);
        }else{
            frontRightMotor.setPower(0);
        }
        if(gamepad1.a){
            backRightMotor.setPower(1);
        }else{
            backRightMotor.setPower(0);
        }
        if(gamepad1.x){
            backLeftMotor.setPower(1);
        }else{
            backLeftMotor.setPower(0);
        }
    }
}

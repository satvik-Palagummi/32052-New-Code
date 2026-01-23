package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sensor.Balls;
import org.firstinspires.ftc.teamcode.Intake.Spinner;
import org.firstinspires.ftc.teamcode.MecanumDrive.Nightcall;
import org.firstinspires.ftc.teamcode.Outtake.PushServo;
import org.firstinspires.ftc.teamcode.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Outtake.TurretLocalization;
import org.firstinspires.ftc.teamcode.Sensor.Colorsensor;
import org.firstinspires.ftc.teamcode.Sensor.Limelight;

@TeleOp
public class Frankenstein extends LinearOpMode {
    private int[] sorted;
    private int Id;
    private final Nightcall nightcall = new Nightcall();
    private final Turret turret = new Turret();
    private final TurretLocalization turretLocalization = new TurretLocalization();
    private final Spinner spinner = new Spinner();
    private final PushServo pushServo = new PushServo();
    private final Limelight limelight = new Limelight();
    private final Colorsensor colorsensor = new Colorsensor();
    private final Balls balls = new Balls();
    private ElapsedTime time = new ElapsedTime();
    private int shootingPos = 0;
    private double endGameStart;
    private boolean isEndGame;
    private boolean shooting;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleDriving();
            handleShooter();
            handleIntake();
            handleLocalization();
            displayTelemetry();
            startGame();
        }
    }
    private void initHardware() {
        nightcall.initialize(hardwareMap);
        spinner.initSpinner(hardwareMap);
        turret.initTurret(hardwareMap);
        pushServo.initPushServos(hardwareMap);
        turretLocalization.initTurretLocalization(hardwareMap);
        colorsensor.initColorSensor(hardwareMap);
        limelight.initLimelight(hardwareMap);
    }

    public void handleDriving(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        addTelemetry("X-value", x);
        addTelemetry("Y-value", y);
        addTelemetry("Rotation", rx);
        boolean slow = false;
        boolean shooting = false;
        if(gamepad1.left_trigger > 0.1){
            slow = true;
        }
        if(gamepad1.left_trigger < 0.1){
            slow = false;
        }
        nightcall.drive(x, y, rx, slow);
        if(gamepad1.ps){
            nightcall.resetYaw();
        }
        if(endGameStart >= getRuntime() && !isEndGame){
            gamepad1.rumbleBlips(3);
            isEndGame = true;
        }
    }
    public void startGame(){
        endGameStart = getRuntime() + 105;
    }

    public void handleShooter(){
        if(gamepad1.right_trigger>0.1){
            pushServo.propel(shootingPos);
        }else{
            pushServo.retract(shootingPos);
        }
        if(gamepad2.right_trigger>0.1){
            turret.startOuttake();
        }else{
            turret.stopOuttake();
        }
        if(gamepad1.cross){
            LLResult result = limelight.updateLimelight();
            Id = limelight.scanMotif(result);
            balls.setMotif(Id);
            if(balls.getFullMotif() != null){
                limelight.stop();
            }
        }
        if(gamepad2.left_bumper){
            limelight.updateLimelight();
            limelight.scanGoal();
            if(limelight.getTx()>0){
                nightcall.leftOrient();
            }else if(limelight.getTx()<0){
                nightcall.rightOrient();
            }else{
                nightcall.cutPower();
                limelight.stop();
            }
        }
        if(gamepad2.y){
            turret.setPower(1420);
        }
        if(gamepad2.a){
            turret.setPower(1640);
        }
    }
    public void handleLocalization(){
        turretLocalization.moveToLeft(gamepad2.dpad_left);
        turretLocalization.moveToMiddle(gamepad2.dpad_up);
        turretLocalization.moveToRight(gamepad2.dpad_right);
        shootingPos = turretLocalization.getTurretPos();
        if(shootingPos == 0){
            pushServo.retract(1);
            pushServo.retract(2);
        }
        if(shootingPos == 1){
            pushServo.retract(0);
            pushServo.retract(2);
        }
        if(shootingPos == 2){
            pushServo.retract(0);
            pushServo.retract(1);
        }
    }
    public void handleIntake(){
        spinner.Intake(gamepad1.square);
    }
    private void displayTelemetry() {
        addTelemetry("Turret Position", turretLocalization.getTurretPos());
        addTelemetry("Is Left Flywheel Motor Shooting", turret.getTurretLPower());
        addTelemetry("Is Right Flywheel Motor Shooting", turret.getTurretRPower());
        addTelemetry("TurretL Velocity", turret.getTurretLVelocity());
        addTelemetry("TurretR Velocity", turret.getTurretRVelocity());
        addTelemetry("Right Spinner Power: ", spinner.getSpinnerRight());
        addTelemetry("Left Spinner Power: ", spinner.getSpinnerLeft());
        addTelemetry("April Tag ID: ", Limelight.detectedTagId);
        addTelemetry("red = ", colorsensor.getRed());
        addTelemetry("blue = ",  colorsensor.getBlue());
        addTelemetry("green = ", colorsensor.getGreen());
        addTelemetry("Current Balls", balls.getCurrentBalls());
        addTelemetry("Motif", balls.getFullMotif());
        addTelemetry("Sorted ", sorted);
        //telemetry.addData("Motif: ", balls.getFullMotif());
        telemetry.update();
    }
    public void addTelemetry(String value, int position){
        telemetry.addData(value, position);

    }
    public void addTelemetry(String value, double num){
        telemetry.addData(value, num);
    }
    public void addTelemetry(String value, int[] num){
        telemetry.addData(value, num);
    }


}

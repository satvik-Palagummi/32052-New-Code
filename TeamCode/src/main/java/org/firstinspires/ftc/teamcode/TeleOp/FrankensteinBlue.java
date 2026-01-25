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
import org.firstinspires.ftc.teamcode.pedroPathing.Auton.FirstAuton;

@TeleOp
public class FrankensteinBlue extends LinearOpMode {
    public enum Position {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        FIRSTPOS,
        SECPOS,
        THIRDPOS
    }
    Position pos;
    private double toTheLeft = -1.5;
    private double toTheRight = 1.5;
    private int[] sorted;
    private int Id;
    private boolean allThree;
    private final Nightcall nightcall = new Nightcall();
    private final Turret turret = new Turret();
    private final TurretLocalization turretLocalization = new TurretLocalization();
    private final Spinner spinner = new Spinner();
    private final PushServo pushServo = new PushServo();
    private final Limelight limelight = new Limelight();
    private final Colorsensor colorsensor = new Colorsensor();
    private final Balls balls = new Balls();
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime gameTime = new ElapsedTime();
    private int shootingPos = 0;
    private double endGameStart;
    private boolean isEndGame;
    private boolean shooting;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        time.reset();
        pos = Position.FIRSTPOS;
        gameTime.reset();

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
    }
    public void startGame(){
        if(gameTime.seconds() == 110){
            gamepad1.rumble(1000);
        }
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
        if(gamepad1.left_trigger>0.1){
            limelight.updateLimelight();
            limelight.scanGoal();
            if(limelight.resultWorks()&& limelight.getTx()>toTheRight){
                nightcall.leftOrient();
            }else if(limelight.resultWorks()&& limelight.getTx()<toTheLeft){
                nightcall.rightOrient();
            }else{
                nightcall.cutPower();
            }
        }
        if(gamepad2.y){
            turret.setPower(1420);
            toTheLeft = -1.5;
            toTheRight = 1.5;
        }
        if(gamepad2.a){
            turret.setPower(1680);
            toTheLeft = 2.5;
            toTheRight = 4.5;
        }
        if(gamepad2.optionsWasPressed()){
            time.reset();
        }
        if(gamepad1.rightBumperWasPressed()){
            turret.startOuttake();
            allThree = false;
            time.reset();
            pos = Position.FIRSTPOS;
        }
        if(!allThree){
            switch(pos) {
                case FIRSTPOS:
                    turretLocalization.setPos(0);
                    if (time.seconds() > 1.0) {
                        pushServo.propel(0);
                    }
                    if(time.seconds()>1.5){
                        pushServo.retract(0);
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    turretLocalization.setPos(1);
                    if (time.seconds() > 1.0) {
                        pushServo.propel(1);
                    }
                    if(time.seconds()>1.5){
                        pushServo.retract(1);
                        setPathState(Position.THIRDPOS);
                    }
                    break;
                case THIRDPOS:
                    turretLocalization.setPos(2);
                    if (time.seconds() > 1.0) {
                        pushServo.propel(2);
                    }
                    if(time.seconds()>1.5){
                        pushServo.retract(2);
                        allThree = true;
                    }
                    break;
            }
        }
    }
    public void setPathState(Position newState) {
        pos = newState;
        time.reset();
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
        if(gamepad2.circle){
            spinner.reverse();
        }
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
        /*
        addTelemetry("red = ", colorsensor.getRed());
        addTelemetry("blue = ",  colorsensor.getBlue());
        addTelemetry("green = ", colorsensor.getGreen());
         */
        addTelemetry("Limelight X", limelight.getTx());
        addTelemetry("Current Balls", balls.getCurrentBalls());
        addTelemetry("Motif", balls.getFullMotif());
        addTelemetry("Sorted ", sorted);
        addTelemetry("Time", time.seconds());
        addTelemetry("Game Time", gameTime.seconds());
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

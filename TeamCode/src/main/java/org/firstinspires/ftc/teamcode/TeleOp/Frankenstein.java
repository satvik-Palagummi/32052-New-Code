package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Intake.Ball;
import org.firstinspires.ftc.teamcode.Intake.Spinner;
import org.firstinspires.ftc.teamcode.MecanumDrive.Nightcall;
import org.firstinspires.ftc.teamcode.Outtake.PushServo;
import org.firstinspires.ftc.teamcode.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Outtake.TurretLocalization;
import org.firstinspires.ftc.teamcode.Sensor.ColorSensor;

@TeleOp
public class Frankenstein extends LinearOpMode {

    private final Nightcall nightcall = new Nightcall();
    private final Turret turret = new Turret();
    private final TurretLocalization turretLocalization = new TurretLocalization();
    private final Spinner spinner = new Spinner();
    private final PushServo pushServo = new PushServo();
    private ElapsedTime time = new ElapsedTime();
    private int shootingPos = 0;
    private double endGameStart;
    private boolean isEndGame;


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
    }

    public void handleDriving(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        addTelemetry("X-value", x);
        addTelemetry("Y-value", y);
        addTelemetry("Rotation", rx);
        boolean slow = false;
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
            turret.startOuttake();
        }else{
            turret.stopOuttake();
        }
        if(gamepad1.right_bumper){
            pushServo.propel(shootingPos);
        }else{
            pushServo.retract(shootingPos);
        }
    }
    public void handleLocalization(){
        /*if(gamepad1.right_bumper){
            if(shootingPos == 0){
                shootingPos = 1;
                addTelemetry("Turret Position RNFOR1", shootingPos);
            }else if(shootingPos == 1){
                shootingPos = 2;
                addTelemetry("Turret Position RNFOR2", shootingPos);
            }
            turretLocalization.setPos(shootingPos);
            telemetry.update();
            while(time.seconds()<5){
                turret.startOuttake();
            }
        }
        if(gamepad1.left_bumper){
            if(shootingPos == 2){
                shootingPos = 1;
                addTelemetry("Turret Position RNFOR1", shootingPos);
            }else if(shootingPos == 1){
                shootingPos = 0;
                addTelemetry("Turret Position RNFOR0", shootingPos);
            }
            turretLocalization.setPos(shootingPos);
            telemetry.update();
        }*/
        if(gamepad1.dpad_left){
            shootingPos = 0;
            turretLocalization.setPos(shootingPos);
        }
        if(gamepad1.dpad_up){
            shootingPos = 1;
            turretLocalization.setPos(shootingPos);
        }
        if(gamepad1.dpad_right){
            shootingPos = 2;
            turretLocalization.setPos(shootingPos);
        }
    }
    public void handleIntake(){
        if(gamepad1.square) {
            spinner.startIntake();
        }else{
            spinner.stopIntake();
        }
    }
    private void displayTelemetry() {
        addTelemetry("Turret Position", turretLocalization.getTurretPos());
        addTelemetry("Is Shooting", turret.getTurret());
        addTelemetry("Right Spinner Power: ", spinner.getSpinnerRight());
        addTelemetry("Left Spinner Power: ", spinner.getSpinnerLeft());
        telemetry.update();
    }
    public void addTelemetry(String value, int position){
        telemetry.addData(value, position);

    }
    public void addTelemetry(String value, double num){
        telemetry.addData(value, num);
    }


}

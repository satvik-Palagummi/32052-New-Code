package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Intake.Spinner;
import org.firstinspires.ftc.teamcode.MecanumDrive.Nightcall;
import org.firstinspires.ftc.teamcode.Outtake.PushServo;
import org.firstinspires.ftc.teamcode.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Outtake.TurretLocalization;
import org.firstinspires.ftc.teamcode.Sensor.Balls;
import org.firstinspires.ftc.teamcode.Sensor.Colorsensor;
import org.firstinspires.ftc.teamcode.Sensor.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

/**
 * Base template for all autonomous OpModes.
 * Contains common functionality for path following, timing, and subsystem control.
 */
public abstract class AutonTemplate extends OpMode {
    protected Follower follower;
    protected Timer pathTimer, actionTimer, opModeTimer;

    protected Turret turret;
    protected PushServo pushServo;
    protected Spinner spinner;
    protected Nightcall nightcall;
    protected TurretLocalization turretLocalization;
    protected Balls balls;
    protected Colorsensor colorsensor;
    protected Limelight limelight;
    protected int[] sorted;

    /**
     * Set the current path state and reset the path timer
     */

    /**
     * Wait for a specified amount of time while keeping subsystems updated
     */
    protected void wait(double time) {
        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < time) {
            follower.update();
            telemetry.addData("Turret Location", turretLocalization.getTurretPos());
            telemetry.addData("Spinner Left:", spinner.getSpinnerLeft());
            telemetry.addData("Spinner Right:", spinner.getSpinnerRight());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
    protected void scanBalls(){
        int[] current = new int[3];
        for(int i = 0; i<3; i++){
            turretLocalization.setPos(i);
            if(i==0) {
                wait(0.1);
            }else{
                wait(0.3);
            }
            pushServo.propelScan(i);
            wait(0.2);
            current[i] = colorsensor.getColorVal();
        }
        balls.setCurrent(current);
    }
    protected void scan(){
        limelight.updateLimelight();
        limelight.scanMotif();
        balls.setMotif();
    }

    /**
     * Execute autonomous shooting sequence (3 balls)
     */
    protected void autonShoot(int i) {
        turret.startOuttake();
        wait(0.1);
        turretLocalization.setPos(i);
        wait(2.0);
        if(turret.getTurretLVelocity() == 1420 && turret.getTurretRVelocity() == 1420) {pushServo.propel(i);}
        wait(0.5);
        pushServo.retract(i);
        if(i==2){
            turret.stopOuttake();
        }
    }
    protected void autonShoot2() {
        /*
        for(int i = 0; i<3; i++){
            turret.startOuttake();
            wait(0.5);
            turretLocalization.setPos(i);
            wait(1.0);
            if(turret.getTurretVelocity() == 1420) {pushServo.propel(i);}
            wait(2.0);
            pushServo.retract(i);
            if(i==2){
                turret.stopOuttake();
            }
        }

         */
    }
    protected void autonShoot2_5() {
        /*
        for(int i = 0; i<3; i++){
            turret.startOuttake();
            wait(0.5);
            turretLocalization.setPos(i);
            wait(1.0);
            if(turret.getTurretVelocity() == 1640) {pushServo.propel(i);}
            wait(2.0);
            pushServo.retract(i);
            if(i==2){
                turret.stopOuttake();
            }
        }

         */
    }


    protected void autonShoot3() {

        turret.startOuttake();
        if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
            sorted = balls.sortBalls();
        }
        for (int i = 0; i < 3; i++) {
            wait(1.0);
            turretLocalization.setPos(sorted[i]);
            wait(0.5);
            if(turret.getTurretLVelocity() == 1420 && turret.getTurretRVelocity()==1420){pushServo.propel(i);}
            wait(1.0);
            pushServo.retract(i);
            if(i==2){
                turret.stopOuttake();
            }
        }
    }
    protected void autonShoot3_5() {
        turret.startOuttake();
        turret.setPower(1640);
        if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
            sorted = balls.sortBalls();
        }
        for (int i = 0; i < 3; i++) {
            wait(1.0);
            turretLocalization.setPos(sorted[i]);
            wait(2.0);
            if(turret.getTurretLVelocity() == 1640 && turret.getTurretRVelocity() == 1640){pushServo.propel(i);}
            wait(1.5);
            pushServo.retract(i);
            if(i==2){
                turret.stopOuttake();
            }
        }
    }


    protected void runAutonIntake() {
        spinner.startIntake();
    }

    /**
     * Stop intake motors
     */
    protected void stopAutonIntake() {
        spinner.stopIntake();
    }

    /**
     * Build paths - must be implemented by subclass
     */
    protected abstract void buildPaths();

    /**
     * Update autonomous path state machine - must be implemented by subclass
     */
    protected abstract void statePathUpdate();

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        turret = new Turret();
        spinner = new Spinner();
        turretLocalization = new TurretLocalization();
        nightcall = new Nightcall();
        pushServo = new PushServo();
        balls = new Balls();
        limelight = new Limelight();
        colorsensor = new Colorsensor();
        nightcall.initialize(hardwareMap);
        spinner.initSpinner(hardwareMap);
        pushServo.initPushServos(hardwareMap);
        turretLocalization.initTurretLocalization(hardwareMap);
        turret.initTurret(hardwareMap);
        limelight.initLimelight(hardwareMap);
        colorsensor.initColorSensor(hardwareMap);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opModeTimer.resetTimer();
        follower.activateAllPIDFs();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Common telemetry
        telemetry.addData("Shooter Velocity", turret.getTurretLVelocity());
        telemetry.update();
        /*
        shooter.periodic();
        spindexer.periodic();
        intake.periodic();
         */
    }

    @Override
    public void stop() {}
}
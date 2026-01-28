package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.teamcode.TeleOp.FrankensteinBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

/**
 * Base template for all autonomous OpModes.
 * Contains common functionality for path following, timing, and subsystem control.
 */
public abstract class AutonTemplate extends OpMode {
    public enum Position {
        FIRSTPOS,
        SECPOS,
        THIRDPOS
    }
    Position pos;
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
    protected int id;

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
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
    /*
    protected void scanBalls(){
        int[] current = new int[3];
        for(int i = 0; i<3; i++){
            turretLocalization.setPos(i);
            if(i==0) {
                wait(2.0);
            }else{
                wait(3.0);
            }
            pushServo.propelScan(i);
            wait(3.0);
            current[i] = colorsensor.getColorVal();
        }
        balls.setCurrent(current);
    }

     */
    protected boolean checkGreen(){
        if(balls.getCurrentBalls() != null){
            int[] current = balls.getCurrentBalls();
            for(int i = 0; i<3; i++){
                turretLocalization.setPos(i);
                wait(0.35);
                pushServo.propelScan(i);
                wait(1.0);
                if(colorsensor.getColorVal()== 0){
                    pushServo.retract(i);
                    return true;
                }
                pushServo.retract(i);
                if(i==2){
                    return false;
                }
            }
        }
        return false;
    }
    protected void scan(){
        LLResult result = limelight.updateLimelight();
        id = limelight.scanMotif(result);
        balls.setMotif(id);
        if(balls.getFullMotif() != null){
            limelight.stop();
        }
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
    /*
    protected boolean scanToShoot(int i){
        turretLocalization.setPos(i);
        wait(0.3);
        pushServo.propelScan(i);
        wait(0.2);
        if(balls.getMotif(i)== colorsensor.getColorVal()){
            pushServo.propel(i);
            wait(1.0);
            pushServo.retract(i);
            return true;
        }
        pushServo.retract(i);
        return false;
    }
    protected void autonShoot2() {
        boolean[] current = {false, false, false};
        int[] giveUpArray = {0,1,2};
        int currentPos;
        boolean giveUp = false;
        for(int i = 0; i<3;i++){
            turret.setPower(1420);
            turret.startOuttake();
            if(!giveUp) {
                for (int j = 0; j < 3; j++) {
                    if(i>0){
                        if(current[0]){
                            if(current[1]){
                                if(current[2]){
                                    break;
                                }else{
                                    if(scanToShoot(2)) {
                                        current[2] = true;
                                        break;
                                    }
                                }
                            }else{
                                if(scanToShoot(1)) {
                                    current[1] = true;
                                    break;
                                }
                            }
                        }else{
                            if(scanToShoot(0)) {
                                current[0] = true;
                                break;
                            }
                        }
                    }else{
                        if(scanToShoot(j)) {
                            current[j] = true;
                            break;
                        }
                    }
                }
            }
        }
    }

     */
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
        turret.setPower(1420);
        turret.startOuttake();
        turretLocalization.setPos(0);
        if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
            sorted = balls.sortBalls();
            for (int i = 0; i < 3; i++) {
                wait(0.2);
                turretLocalization.setPos(sorted[i]);
                wait(0.4);
                pushServo.propel(sorted[i]);
                wait(0.3);
                pushServo.retract(sorted[i]);
                if(i==2){
                    turret.stopOuttake();
                }
            }
        }
    }

    protected void autonShoot3_5() {
        turret.startOuttake();
        turret.setPower(1680);
        if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
            sorted = balls.sortBalls();
            for (int i = 0; i < 3; i++) {
                wait(1.0);
                turretLocalization.setPos(sorted[i]);
                wait(2.0);
                if(turret.getTurretLVelocity() == 1640 && turret.getTurretRVelocity() == 1640){pushServo.propel(sorted[i]);}
                wait(1.5);
                pushServo.retract(sorted[i]);
                if(i==2){
                    turret.stopOuttake();
                }
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
        telemetry.addData("Timer", actionTimer.getElapsedTimeSeconds());
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
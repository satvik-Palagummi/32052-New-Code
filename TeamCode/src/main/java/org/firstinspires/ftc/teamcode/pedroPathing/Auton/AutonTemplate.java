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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
            telemetry.update();
        }
    }

    /**
     * Execute autonomous shooting sequence (3 balls)
     */
    protected void autonShoot(int i) {
        turret.startOuttake();
        wait(.3);
        turretLocalization.setPos(i);
        wait(2.0);
        pushServo.propel(i);
        wait(1.0);
        pushServo.retract(i);
    }

    protected void autonShoot2() {
        turret.startOuttake();
        for (int i = 0; i < 3; i++) {
            wait(.3);
            turretLocalization.setPos(i);
            wait(1.5);
            pushServo.propel(i);
            wait(1.0);
            pushServo.retract(i);
        }
        turret.stopOuttake();
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
        nightcall.initialize(hardwareMap);
        spinner.initSpinner(hardwareMap);
        pushServo.initPushServos(hardwareMap);
        turretLocalization.initTurretLocalization(hardwareMap);
        turret.initTurret(hardwareMap);
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
        telemetry.addData("Shooter Power", turret.getTurret());
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
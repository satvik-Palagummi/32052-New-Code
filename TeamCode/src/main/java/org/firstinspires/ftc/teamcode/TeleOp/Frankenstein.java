package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outtake.HoodMovement;
import org.firstinspires.ftc.teamcode.Sensor.Balls;
import org.firstinspires.ftc.teamcode.Intake.Spinner;
import org.firstinspires.ftc.teamcode.MecanumDrive.Nightcall;
import org.firstinspires.ftc.teamcode.Outtake.PushServo;
import org.firstinspires.ftc.teamcode.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Outtake.TurretLocalization;
import org.firstinspires.ftc.teamcode.Sensor.Colorsensor;
import org.firstinspires.ftc.teamcode.Sensor.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

public abstract class Frankenstein extends LinearOpMode {
    public abstract int getLimelightPipeline();
    public abstract double toTheLeftFar();
    //Blue -2, Red -4
    public abstract double toTheRightFar();

    public enum Position {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        FIRSTPOS,
        SECPOS,
        THIRDPOS
    }
    Position pos;
    Position sortedPos;
    Position farPos;
    Position sortedFarPos;
    protected double toTheLeft = -1.5;
    protected double toTheRight = 1.5;
    protected int[] sorted;
    protected int Id;
    protected Follower follower;
    protected boolean allThree = true;
    protected final Nightcall nightcall = new Nightcall();
    protected final Turret turret = new Turret();
    protected final TurretLocalization turretLocalization = new TurretLocalization();
    protected final Spinner spinner = new Spinner();
    protected final PushServo pushServo = new PushServo();
    protected final Limelight limelight = new Limelight();
    protected final HoodMovement hoodMovement = new HoodMovement();
    protected final Colorsensor colorsensor = new Colorsensor();
    protected final Balls balls = new Balls();
    protected ElapsedTime time = new ElapsedTime();
    protected ElapsedTime gameTime = new ElapsedTime();
    protected int shootingPos = 0;
    protected boolean fromFar;
    protected boolean outtake;
    protected int current = 1;
    protected boolean motifSet;
    protected boolean slow;
    protected double distance;
    protected boolean startSort = false;
    protected boolean farShot;
    protected boolean intook;
    protected boolean reached;
    protected double timing = 0;
    protected double lastDistance;
    protected double F = 12.1623;
    protected double P = 100;
    protected double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    protected int stepIndex  = 1;
    protected double highVelocity = 1640;
    protected double curTargetVelocity = highVelocity;
    protected double lowVelocity = 900;
    protected double errorL;
    protected double errorR;
    protected boolean autoAim;


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
            endGame();
        }
    }
    private void initHardware() {
        //Initialize all hardware. IF you face a null error not initializing it is probably why.
        follower = Constants.createFollower(hardwareMap);
        nightcall.initialize(hardwareMap);
        spinner.initSpinner(hardwareMap);
        turret.initTurret(hardwareMap);
        pushServo.initPushServos(hardwareMap);
        turretLocalization.initTurretLocalization(hardwareMap);
        colorsensor.initColorSensor(hardwareMap);
        limelight.initLimelight(hardwareMap);
        hoodMovement.initHoodServo(hardwareMap);
        hoodMovement.setHood(0.4);
        turretLocalization.setPos(1);
    }

    public void handleDriving(){
        //Main Mecanum teleop
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        addTelemetry("X-value", x);
        addTelemetry("Y-value", y);
        addTelemetry("Rotation", rx);
        //Sets driving to a slower speed of about speed
        if(gamepad1.left_bumper){
            slow = true;
        }else{
            slow = false;
        }
        nightcall.drive(x, y, rx, slow);
        if(gamepad1.start){
            nightcall.resetYaw();
        }
    }
    public void endGame(){
        if(gameTime.seconds() == 110){
            gamepad2.rumble(1000);
        }
    }

    public void handleShooter(){
        /**
         * Propels artifact with servo for the position that the Shooter is currently set to.
         */
        if(gamepad1.right_trigger>0.1){
            pushServo.propel(shootingPos);
        }else if(gamepad1.right_trigger<0.3 &&allThree){
            pushServo.retract(shootingPos);
        }
        /**Starts Flywheel

         */
        if(gamepad2.rightBumperWasPressed()&& allThree){
            outtake = !outtake;
        }
        turret.startOuttake(outtake);
        if(gamepad2.squareWasPressed()){
            if(!motifSet){
                motifSet = true;
            }
        }
        /**Gives Player 2 the option to restart setting the motif if they messed up.

         */
        /*
        if(gamepad2.dpadDownWasPressed()&& motifSet){
            motifSet = false;
            balls.setMotif(0);
            current = 0;
        }

         */
        /**Left Bumper toggles through the arrays that the Motif could be set to:
        [Purple, Purple, Green] (1,1,0),
        [Purple, Green, Purple] (1,0,1),
        [Green, Purple, Purple] (0,1,1)

         */
        if(gamepad2.leftBumperWasPressed()){
            if(!motifSet) {
                current++;
                if (current % 3 == 1) {
                    balls.setMotif(21);
                } else if (current % 3 == 2) {
                    balls.setMotif(22);
                } else if (current % 3 == 0) {
                    balls.setMotif(23);
                }
            }
        }
        /**
         * Gets interpolated shot parameters based on distance.
         * @param distance Distance to the target in cm.
         * @return ShotParams containing flywheel velocity and hood position.
         */
        if(gamepad1.left_trigger>0.1){
            limelight.setPipeline(getLimelightPipeline());
            limelight.updateLimelight();
            limelight.scanGoal();
            if(limelight.resultWorks()&&limelight.getTx()>toTheRight){
                nightcall.rightOrient();
            }else if(limelight.resultWorks()&&limelight.getTx()<toTheLeft){
                nightcall.leftOrient();
            }else{
                nightcall.cutPower();
            }
            distance = limelight.getDistance(limelight.getTa());
            lastDistance = distance;
        }
        /**Look-up table on flywheel, using distance calculated from Limelight April Tag Area size formula.
         */
        if(distance == 0){
        } else if(distance >110 && distance < 150){
            turret.setPower(1500);
            curTargetVelocity = 1500;
            farShot = false;
            toTheLeft = -1.75;
            toTheRight = 1.75;
            distance = 0;
            hoodMovement.setHood(0.6);
        }else if(distance >150 && distance < 160){
            turret.setPower(1520);
            curTargetVelocity = 1520;
            farShot = false;
            toTheLeft = -1.75;
            toTheRight = 1.75;
            distance = 0;
            hoodMovement.setHood(0.5);
        }else if(distance>160 && distance < 175){
            turret.setPower(1530);
            curTargetVelocity = 1530;
            farShot = false;
            toTheLeft = -1.75;
            toTheRight = 1.75;
            distance = 0;
            hoodMovement.setHood(0.45);
        }else if(distance>175 && distance < 210){
            turret.setPower(1530);
            curTargetVelocity = 1530;
            farShot = false;
            toTheLeft = -1.75;
            toTheRight = 1.75;
            distance = 0;
            hoodMovement.setHood(0.5);
        } else if(distance>290 && distance < 310){
            turret.setPower(1860);
            curTargetVelocity = 1860;
            farShot = true;
            toTheLeft = toTheLeftFar();
            toTheRight = toTheRightFar();
            distance = 0;
            hoodMovement.setHood(0.3);
        } else if(distance>310 && distance < 325){
            turret.setPower(1880);
            curTargetVelocity = 1880;
            farShot = true;
            toTheLeft = toTheLeftFar();
            toTheRight = toTheRightFar();
            distance = 0;
            hoodMovement.setHood(0.3);
        }else if(distance>325 && distance < 335){
            turret.setPower(1920);
            curTargetVelocity = 1920;
            farShot = true;
            toTheLeft = toTheLeftFar();
            toTheRight = toTheRightFar();
            distance = 0;
            hoodMovement.setHood(0.3);
        }else if(distance>335 && distance < 355){
            turret.setPower(1930);
            curTargetVelocity = 1930;
            farShot = true;
            toTheLeft = toTheLeftFar();
            toTheRight = toTheRightFar();
            distance = 0;
            hoodMovement.setHood(0.3);
        }
        if(gamepad1.rightBumperWasPressed()){
            turret.startOuttake();
            allThree = false;
            time.reset();
            pos = Position.FIRSTPOS;
            sortedPos = Position.FIRSTPOS;
            farPos = Position.FIRSTPOS;
            sortedFarPos = Position.FIRSTPOS;
            if(turretLocalization.getTurretPos()==2){
                fromFar = true;
            }
        }
        /**Enables sorting in Tele-op after motif has been set.

         */
        if(gamepad1.backWasPressed()){
            startSort = !startSort;
        }
        //Changes velocity during flywheel PIDF tuning.
        if(gamepad2.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            }else{
                curTargetVelocity = highVelocity;
            }
        }
        if(gamepad1.yWasPressed()){
            curTargetVelocity+=10;
        }
        if(gamepad1.aWasPressed()) {
            curTargetVelocity-=10;
        }
        turret.setPower(curTargetVelocity);

        if(gamepad2.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad2.dpadLeftWasPressed() && !gamepad2.back){
            F += stepSizes[stepIndex];
        }
        if(gamepad2.dpadRightWasPressed() && !gamepad2.back){
            F -= stepSizes[stepIndex];
        }
        if(gamepad2.dpadUpWasPressed() &&!gamepad2.back){
            P += stepSizes[stepIndex];
        }
        if(gamepad2.dpadDownWasPressed()&&!gamepad2.back){
            P -= stepSizes[stepIndex];
        }

        double curLVelocity = turret.getTurretLVelocity();
        double curRVelocity = turret.getTurretRVelocity();
        errorL = curTargetVelocity - curLVelocity;
        errorR = curTargetVelocity - curRVelocity;
        turret.setPIDF(P, F);
        /**If intake button has been released, grab color sensor values.


        */
        if(intook){
            int[] temp = colorsensor.getCurrent();
            balls.setCurrent(temp);
            if(((Arrays.equals(balls.getCurrentBalls(), new int[]{0, 1, 1}))
                    || (Arrays.equals(balls.getCurrentBalls(), new int[]{1, 0, 1}))
                    || (Arrays.equals(balls.getCurrentBalls(), new int[]{1, 1, 0})))
                    &&(!(Arrays.equals(balls.getFullMotif(), new int[]{-1,-1,-1})))
                    &&startSort){
                sorted = balls.sortBalls();
            }
            intook = false;
        }
        if(gamepad2.back){
            if (gamepad2.dpadDownWasPressed()) {
                hoodMovement.adjustHoodDown();
            }
            if(gamepad2.dpadUpWasPressed()){
                hoodMovement.adjustHoodUp();
            }
        }

        //AUTO SHOOT FOR REGULAR CLOSE SHOTS
        //
        //
        //
        if(!allThree && sorted == null && !farShot){
            switch(pos) {
                case FIRSTPOS:
                    if(balls.getCurrentSpecPos(0)!=-1){
                        turretLocalization.setPos(0);
                        if (turretLocalization.getTurretArrived(0) &&time.seconds()<0.5) {
                            pushServo.propel(0);
                        }else if(time.seconds()>0.5){
                            pushServo.retract(0);
                            setPathState(Position.SECPOS);
                        }
                    }else{
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    if(balls.getCurrentSpecPos(1)!=-1) {
                        turretLocalization.setPos(1);
                        if (turretLocalization.getTurretArrived(1)&& time.seconds()<0.5) {
                            pushServo.propel(1);
                        }else if(time.seconds()>0.5){
                            pushServo.retract(1);
                            setPathState(Position.THIRDPOS);
                        }
                    }else{
                        setPathState(Position.THIRDPOS);
                    }
                    break;
                case THIRDPOS:
                    if(balls.getCurrentSpecPos(2)!=-1) {
                        turretLocalization.setPos(2);
                        if (turretLocalization.getTurretArrived(2)&&time.seconds()<0.5) {
                            reached = true;
                            pushServo.propel(2);
                        }else if(time.seconds()>0.5){
                            pushServo.retract(2);
                            allThree = true;
                            turretLocalization.setPos(1);
                        }
                    }else{
                        allThree = true;
                        turretLocalization.setPos(1);
                    }
                    break;
            }
        }
        //AUTO SHOOT FOR REGULAR FAR SHOOTING
        //
        //
        //
        if(!allThree && sorted == null && farShot){
            switch(farPos) {
                case FIRSTPOS:
                    if(balls.getCurrentSpecPos(0)!=-1){
                        turretLocalization.setPos(0);
                        if (turretLocalization.getTurretArrived(0)&& time.seconds()<0.55) {
                            pushServo.propel(0);
                        }else if(time.seconds()>0.55){
                            pushServo.retract(0);
                            setPathState(Position.SECPOS);
                        }
                    }else{
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    if(balls.getCurrentSpecPos(1)!=-1) {
                        turretLocalization.setPos(1);
                        if (turretLocalization.getTurretArrived(1)&& time.seconds()<0.55) {
                            pushServo.propel(1);
                        }else if(time.seconds()>0.55){
                            pushServo.retract(1);
                            setPathState(Position.THIRDPOS);
                        }
                    }else{
                        setPathState(Position.THIRDPOS);
                    }
                    break;
                case THIRDPOS:
                    if(balls.getCurrentSpecPos(2)!=-1) {
                        turretLocalization.setPos(2);
                        if (turretLocalization.getTurretArrived(2)&& time.seconds()<0.55) {
                            pushServo.propel(2);
                        }else if(time.seconds()>0.55){
                            pushServo.retract(2);
                            allThree = true;
                            turretLocalization.setPos(1);
                        }
                    }else{
                        allThree = true;
                        turretLocalization.setPos(1);
                    }
                    break;
            }
        }
        //AUTO SHOOT FOR SORTED CLOSE SHOTS
        //
        //
        //
        if(!allThree&&sorted != null && !farShot){
            switch(sortedPos) {
                case FIRSTPOS:
                    turretLocalization.setPos(sorted[0]);
                    if (turretLocalization.getTurretArrived(sorted[0])&& time.seconds()<0.55) {
                        pushServo.propel(sorted[0]);
                    }else if(time.seconds()>0.55){
                        pushServo.retract(sorted[0]);
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    turretLocalization.setPos(sorted[1]);
                    if (turretLocalization.getTurretArrived(sorted[1])&& time.seconds()<0.55) {
                        pushServo.propel(sorted[1]);
                    }else if(time.seconds()>0.55){
                        pushServo.retract(sorted[1]);
                        setPathState(Position.THIRDPOS);
                    }
                    break;
                case THIRDPOS:
                    turretLocalization.setPos(sorted[2]);
                    if (turretLocalization.getTurretArrived(sorted[2])&& time.seconds()<0.55) {
                        pushServo.propel(sorted[2]);
                    }else if(time.seconds()>0.55){
                        pushServo.retract(sorted[2]);
                        allThree = true;
                        sorted = null;
                        turretLocalization.setPos(1);
                    }
                    break;
            }
        }
        //AUTO SHOOT FOR SORTED FAR SHOTS
        //
        //
        //
        if(!allThree && sorted != null && farShot){
            switch(sortedFarPos) {
                case FIRSTPOS:
                    turretLocalization.setPos(sorted[0]);
                    if (turretLocalization.getTurretArrived(sorted[0])&& time.seconds()<0.6) {
                        pushServo.propel(sorted[0]);
                    }else if(time.seconds()>0.6){
                        pushServo.retract(sorted[0]);
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    turretLocalization.setPos(sorted[1]);
                    if (turretLocalization.getTurretArrived(sorted[1])&& time.seconds()<0.6) {
                        pushServo.propel(sorted[1]);
                    }else if(time.seconds()>0.6){
                        pushServo.retract(sorted[1]);
                        setPathState(Position.THIRDPOS);
                    }
                    break;
                case THIRDPOS:
                    turretLocalization.setPos(sorted[2]);
                    if (turretLocalization.getTurretArrived(sorted[2])&& time.seconds()<0.6){
                        pushServo.propel(sorted[2]);
                    }else if(time.seconds()>0.6){
                        pushServo.retract(sorted[2]);
                        allThree = true;
                        sorted = null;
                        turretLocalization.setPos(1);
                    }
                    break;
            }
        }
    }
    public void handleFollower(){
        follower.update();
    }
    public void setPathState(Position newState) {
        pos = newState;
        sortedPos = newState;
        farPos = newState;
        sortedFarPos = newState;
        time.reset();
    }
    public void handleLocalization(){
        //Sets shooter using linkage to first second and third positions.
        turretLocalization.moveToLeft(gamepad1.dpad_left);
        turretLocalization.moveToMiddle(gamepad1.dpad_up);
        turretLocalization.moveToRight(gamepad1.dpad_right);
        shootingPos = turretLocalization.getTurretPos();
        if(shootingPos == 0 && allThree){
            pushServo.retract(1);
            pushServo.retract(2);
        }
        if(shootingPos == 1&& allThree){
            pushServo.retract(0);
            pushServo.retract(2);
        }
        if(shootingPos == 2 &&allThree){
            pushServo.retract(0);
            pushServo.retract(1);
        }


    }

    public void handleIntake(){
        //activate Intake
        spinner.Intake(gamepad1.square);
        // Activate Reverse intake
        if(gamepad1.b){
            spinner.reverse();
        }else if(!gamepad1.square && !gamepad1.b){
            spinner.stopIntake();
        }
        if(gamepad1.squareWasReleased()){
            intook = true;
        }
    }
    private void displayTelemetry() {
        addTelemetry("Current Balls", Arrays.toString(balls.getCurrentBalls()));
        addTelemetry("Motif", Arrays.toString(balls.getFullMotif()));
        addTelemetry("Sorted ", Arrays.toString(sorted));
        addTelemetry("Turret Velocity", (turret.getTurretLVelocity()+ turret.getTurretRVelocity())/2);
        addTelemetry("Hood Position: ", hoodMovement.hoodPos());
        addTelemetry("Limelight Distance", lastDistance);
        addTelemetry("Servo Voltage: ", turretLocalization.getServoFeedback());
        addTelemetry("Path States", pos);
        addTelemetry("Motif Set", motifSet);
        addTelemetry("SORTING ACTIVATED::: ", startSort);
        /*
        addTelemetry("ErrorL", errorL);
        addTelemetry("ErrorR", errorR);
        addTelemetry("P:", P);
        addTelemetry("F: ",F);
        addTelemetry("Step Size", stepSizes[stepIndex]);

        addTelemetry("Green1: ", colorsensor.getGreen1());
        addTelemetry("Blue1: ", colorsensor.getBlue1());
        addTelemetry("ColorVal1", colorsensor.getColorVal1());
        addTelemetry("Green2: ", colorsensor.getGreen2());
        addTelemetry("Blue2: ", colorsensor.getBlue2());
        addTelemetry("ColorVal2", colorsensor.getColorVal2());
        addTelemetry("Green3: ", colorsensor.getGreen3());
        addTelemetry("Blue3: ", colorsensor.getBlue3());
        addTelemetry("ColorVal3", colorsensor.getColorVal3());
        addTelemetry("Turret Position", turretLocalization.getTurretPos());
        addTelemetry("Is Left Flywheel Motor Shooting", turret.getTurretLPower());
        addTelemetry("Is Right Flywheel Motor Shooting", turret.getTurretRPower());
        addTelemetry("Right Spinner Power: ", spinner.getSpinnerRight());
        addTelemetry("Left Spinner Power: ", spinner.getSpinnerLeft());
        addTelemetry("April Tag ID: ", limelight.getDetectedTagId());
        addTelemetry("Limelight X", limelight.getTx());
        addTelemetry("Time", time.seconds());
        addTelemetry("Game Time", gameTime.seconds());
        */

        telemetry.update();
    }
    public void addTelemetry(String value, int position){
        telemetry.addData(value, position);

    }
    public void addTelemetry(String value, String val){
        telemetry.addData(value, val);
    }
    public void addTelemetry(String value, Object num){
        telemetry.addData(value, num);
    }
    public void addTelemetry(String value, int[] num){
        telemetry.addData(value, num);
    }


}

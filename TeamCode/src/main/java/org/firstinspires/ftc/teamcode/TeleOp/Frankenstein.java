package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sensor.Balls;
import org.firstinspires.ftc.teamcode.Intake.Spinner;
import org.firstinspires.ftc.teamcode.MecanumDrive.Nightcall;
import org.firstinspires.ftc.teamcode.Outtake.HoodMovement;
import org.firstinspires.ftc.teamcode.Outtake.PushServo;
import org.firstinspires.ftc.teamcode.Outtake.ShotLookupTable;
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
    protected final HoodMovement hoodMovement = new HoodMovement();
    protected final Limelight limelight = new Limelight();
    protected final Colorsensor colorsensor = new Colorsensor();
    protected final Balls balls = new Balls();
    protected ElapsedTime time = new ElapsedTime();
    protected ElapsedTime gameTime = new ElapsedTime();
    protected int shootingPos = 0;
    protected boolean fromFar;
    protected boolean outtake;
    protected int current = 1;
    protected boolean motifSet;
    protected double distance;
    protected boolean manualMode = false;
    protected boolean farShot;
    protected boolean intook;


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
        follower = Constants.createFollower(hardwareMap);
        nightcall.initialize(hardwareMap);
        spinner.initSpinner(hardwareMap);
        turret.initTurret(hardwareMap);
        pushServo.initPushServos(hardwareMap);
        hoodMovement.initHood(hardwareMap);
        turretLocalization.initTurretLocalization(hardwareMap);
        //colorsensor.initColorSensor(hardwareMap);
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
        if(gamepad1.right_trigger>0.1){
            pushServo.propel(shootingPos);
        }else if(gamepad1.right_trigger<0.3 &&allThree){
            pushServo.retract(shootingPos);
        }
        if(gamepad2.rightBumperWasPressed()&& allThree){
            outtake = !outtake;
        }
        turret.startOuttake(outtake);
        if(gamepad2.squareWasPressed()){
            if(!motifSet){
                motifSet = true;
                addTelemetry("Motif Set", motifSet);
            }
        }
        if(gamepad2.dpadDownWasPressed()&& motifSet){
            motifSet = false;
            balls.setMotif(0);
            current = 0;
        }
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
        }
        if(!manualMode){
            if(distance != 0){
                ShotLookupTable.ShotParams params = ShotLookupTable.getParams(distance);
                turret.setPower(params.flywheelVelocity);
                hoodMovement.setHoodPosition(params.hoodPosition);

                if (distance > 300) {
                    farShot = true;
                    toTheLeft = toTheLeftFar();
                    toTheRight = toTheRightFar();
                } else {
                    farShot = false;
                    toTheLeft = -1.5;
                    toTheRight = 1.5;
                }
                distance = 0;
            }
        }else{
            if(gamepad2.triangle){
                turret.setPower(1420);
                hoodMovement.setHoodPosition(0);
                toTheLeft = -1.5;
                toTheRight = 1.5;
            }
            if(gamepad2.circle){
                turret.setPower(1460);
                hoodMovement.setHoodPosition(350);
                toTheLeft = -1.5;
                toTheRight = 1.5;
            }
            if(gamepad2.cross){
                turret.setPower(1740);
                hoodMovement.setHoodPosition(950);
                toTheLeft = 2.5;
                toTheRight = 4.5;
            }
        }
        if(gamepad2.optionsWasPressed()){
            manualMode = !manualMode;
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
        if(intook){
            int[] temp = colorsensor.getCurrent();
            balls.setCurrent(temp);
            if(((Arrays.equals(balls.getCurrentBalls(), new int[]{0, 1, 1}))
                    || (Arrays.equals(balls.getCurrentBalls(), new int[]{1, 0, 1}))
                    || (Arrays.equals(balls.getCurrentBalls(), new int[]{1, 1, 0})))
                    &&(!(Arrays.equals(balls.getFullMotif(), new int[]{-1,-1,-1})))){
                sorted = balls.sortBalls();
            }
            intook = false;
        }
        if(!allThree && sorted == null && !farShot){
            switch(pos) {
                case FIRSTPOS:
                    if(balls.getCurrentSpecPos(0)!=-1){
                        turretLocalization.setPos(0);
                        if(fromFar) {
                            if (time.seconds() > 0.5) {
                                pushServo.propel(0);
                            }
                            if(time.seconds()>0.85){
                                pushServo.retract(0);
                                setPathState(Position.SECPOS);
                                fromFar = false;
                            }
                        }else {
                            if (time.seconds() > 0.2) {
                                pushServo.propel(0);
                            }
                            if (time.seconds() > 0.5) {
                                pushServo.retract(0);
                                setPathState(Position.SECPOS);
                            }
                        }
                    }else{
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    if(balls.getCurrentSpecPos(1)!=-1) {
                        turretLocalization.setPos(1);
                        if (time.seconds() > 0.3 && time.seconds() < 0.6) {
                            pushServo.propel(1);
                        }
                        if (time.seconds() > 0.6) {
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
                        if (time.seconds() > 0.25 && time.seconds() < 0.60) {
                            pushServo.propel(2);
                        }
                        if (time.seconds() > 0.60) {
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
        if(!allThree && sorted == null && farShot){
            switch(farPos) {
                case FIRSTPOS:
                    if(balls.getCurrentSpecPos(0)!=-1){
                        turretLocalization.setPos(0);
                        if(fromFar) {
                            if (time.seconds() > 0.5) {
                                pushServo.propel(0);
                            }
                            if(time.seconds()>0.85){
                                pushServo.retract(0);
                                setPathState(Position.SECPOS);
                                fromFar = false;
                            }
                        }else {
                            if (time.seconds() > 0.3) {
                                pushServo.propel(0);
                            }
                            if (time.seconds() > 0.6) {
                                pushServo.retract(0);
                                setPathState(Position.SECPOS);
                            }
                        }
                    }else{
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    if(balls.getCurrentSpecPos(1)!=-1) {
                        turretLocalization.setPos(1);
                        if (time.seconds() > 0.4 && time.seconds() < 0.7) {
                            pushServo.propel(1);
                        }
                        if (time.seconds() > 0.7) {
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
                        if (time.seconds() > 0.4 && time.seconds() < 0.70) {
                            pushServo.propel(2);
                        }
                        if (time.seconds() > 0.70) {
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
        if(!allThree&&sorted != null && !farShot){
            switch(sortedPos) {
                case FIRSTPOS:
                    turretLocalization.setPos(sorted[0]);
                    if (time.seconds() > 0.35) {
                        pushServo.propel(sorted[0]);
                    }
                    if(time.seconds()>0.75){
                        pushServo.retract(sorted[0]);
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    turretLocalization.setPos(sorted[1]);
                    if (time.seconds() > 0.35 && time.seconds() < 0.75) {
                        pushServo.propel(sorted[1]);
                    }
                    if(time.seconds()>0.75){
                        pushServo.retract(sorted[1]);
                        setPathState(Position.THIRDPOS);
                    }
                    break;
                case THIRDPOS:
                    turretLocalization.setPos(sorted[2]);
                    if (time.seconds() > 0.35&& time.seconds()<0.75) {
                        pushServo.propel(sorted[2]);
                    }
                    if(time.seconds()>0.75){
                        pushServo.retract(sorted[2]);
                        allThree = true;
                        sorted = null;
                        turretLocalization.setPos(1);
                    }
                    break;
            }
        }
        if(!allThree && sorted != null && farShot){
            switch(sortedFarPos) {
                case FIRSTPOS:
                    turretLocalization.setPos(sorted[0]);
                    if (time.seconds() > 0.35) {
                        pushServo.propel(sorted[0]);
                    }
                    if(time.seconds()>0.75){
                        pushServo.retract(sorted[0]);
                        setPathState(Position.SECPOS);
                    }
                    break;
                case SECPOS:
                    turretLocalization.setPos(sorted[1]);
                    if (time.seconds() > 0.4 && time.seconds() < 0.7) {
                        pushServo.propel(sorted[1]);
                    }
                    if(time.seconds()>0.7){
                        pushServo.retract(sorted[1]);
                        setPathState(Position.THIRDPOS);
                    }
                    break;
                case THIRDPOS:
                    turretLocalization.setPos(sorted[2]);
                    if (time.seconds() > 0.4 && time.seconds() < 0.7) {
                        pushServo.propel(sorted[2]);
                    }
                    if(time.seconds()>0.7){
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
        turretLocalization.moveToLeft(gamepad2.dpad_left);
        turretLocalization.moveToMiddle(gamepad2.dpad_up);
        turretLocalization.moveToRight(gamepad2.dpad_right);
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
        spinner.Intake(gamepad1.square);
        if(gamepad2.left_trigger>0.3){
            spinner.reverse();
        }else if(!gamepad1.square && gamepad2.left_trigger<0.3){
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
        addTelemetry("Hood Position", hoodMovement.getHoodPosition());
        addTelemetry("Limelight Distance", distance);
        /*
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

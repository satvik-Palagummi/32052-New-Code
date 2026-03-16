package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class FirstAutonBlue extends AutonTemplate {
    public enum PathState {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS,
        SCANPOSE,
        SHOOTING,
        SHOOT1_SHOOT2,
        SHOOT2_SHOOT3,
        SHOOT_PRELOAD1, BALLROW_GRABBING1, GRABBING_REVERSAL3,
        SHOOT_PRELOAD2, BALLROW_GRABBING2, LEVER, LEVER2,
        SHOOT_PRELOAD3, BALLROW_GRABBING3
    }
    PathState pathState;

    private final Pose startPose = new Pose(32.700, 135.670, Math.toRadians(0));
    private final Pose scanPose = new Pose(49, 97, Math.toRadians(-25));
    private final Pose scanControl = new Pose(83,63);
    private final Pose shootPose = new Pose(60.000, 83.000, Math.toRadians(45));
    private final Pose grabBalls1 = new Pose(16.000, 83.000, Math.toRadians(0));
    private final Pose grabBalls1Control = new Pose(68.000, 91.500);
    private final Pose grabBalls2 = new Pose(10.000, 60.000, Math.toRadians(0));
    private final Pose grabBalls2Control = new Pose(73.000, 63.000);
    private final Pose hitLever = new Pose(17.000, 72.000, Math.toRadians(-20));
    private final Pose hitLeverFirstSpike = new Pose(16.000, 72.000, Math.toRadians(85));
    private final Pose hitLeverControl = new Pose (62.000, 60.000);
    private final Pose hitLeverFirstSpikeControl = new Pose(45.000, 77.000);
    private final Pose shootPos2Control = new Pose(64.000, 60.000);
    private final Pose grabBalls3 = new Pose(10.000, 36.000, Math.toRadians(5));
    private final Pose grabBalls3Control = new Pose(72.000, 36.000);
    private final Pose shootPose3Orient = new Pose (49.000, 70.000, Math.toRadians(178));
    private final Pose shootPos3Control = new Pose(63, 46);
    private boolean zeroGrab = false;
    private boolean firstGrab = false;
    private boolean secondGrab = false;
    private boolean thirdGrab = false;

    private PathChain StartToShoot,
            ShootPose,
            ShootPos1To2,
            ShootPos2To3,
            shootToBallGrabbing1, LeverPush2, GrabbingReversal1,
            shootToBallGrabbing2, GrabbingReversal2, LeverPush,
            shootToBallGrabbing3, GrabbingReversal3, Reverse3;

    public void buildPaths(){
        // ReachShootingPoint
        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // ShootPose (kept original logic if needed, but updated to use shootPose)
        ShootPose = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, shootPose))
                .setLinearHeadingInterpolation(scanPose.getHeading(), shootPose.getHeading())
                .build();

        // Path3 -> shootToBallGrabbing1
        shootToBallGrabbing1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls1Control, grabBalls1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path10 (first) -> LeverPush2
        LeverPush2 = follower.pathBuilder()
                .addPath(new BezierCurve(grabBalls1, hitLeverFirstSpikeControl, hitLeverFirstSpike))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(85))
                .build();

        // Path5 -> GrabbingReversal1
        GrabbingReversal1 = follower.pathBuilder()
                .addPath(new BezierLine(hitLeverFirstSpike, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(85), Math.toRadians(45))
                .build();

        // Path7 -> shootToBallGrabbing2
        shootToBallGrabbing2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls2Control, grabBalls2))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path9 (first) -> LeverPush
        LeverPush = follower.pathBuilder()
                .addPath(new BezierCurve(grabBalls2, hitLeverControl, hitLever))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        // Path9 (second) -> GrabbingReversal2
        GrabbingReversal2 = follower.pathBuilder()
                .addPath(new BezierCurve(hitLever, shootPos2Control, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(45))
                .build();

        // Path11 -> shootToBallGrabbing3
        shootToBallGrabbing3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls3Control, grabBalls3))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(5))
                .build();

        // Path13 -> GrabbingReversal3
        GrabbingReversal3 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls3, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(5), Math.toRadians(45))
                .build();

        // Path10 (second) -> Reverse3
        Reverse3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, shootPose3Orient))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(178))
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case STARTPOS:
                hoodMovement.setHood(0.43);
                balls.setCurrent(new int[]{1,1,0});
                sorted = balls.sortBalls();
                turretLocalization.setPos(1);
                limelight.setPipeline(8);
                turret.setPower(1530);
                turret.startOuttake();
                allThreeSorted = false;
                follower.followPath(StartToShoot, true);
                setPathState(PathState.SHOOTING);//Resets timer & makes new state
                break;
            case SHOOTING:
                if(!follower.isBusy())
                {
                    stopAutonIntake();
                    autonShoot3Blue();

                    if(!secondGrab && allThreeSorted) {
                        turret.stopOuttake();
                        turretLocalization.setPos(1);
                        follower.setMaxPower(0.7);
                        runAutonIntake();
                        follower.followPath(shootToBallGrabbing2, true);
                        setPathState(PathState.BALLROW_GRABBING2);
                    }else if(!firstGrab && allThreeSorted){
                        turret.stopOuttake();
                        turretLocalization.setPos(1);
                        follower.setMaxPower(0.8);
                        runAutonIntake();
                        follower.followPath(shootToBallGrabbing1, true);
                        setPathState(PathState.BALLROW_GRABBING1);
                    } else if (!thirdGrab && allThreeSorted) {
                        turret.stopOuttake();
                        turretLocalization.setPos(1);
                        follower.setMaxPower(1.0);
                        runAutonIntake();
                        follower.followPath(shootToBallGrabbing3, true);
                        setPathState(PathState.BALLROW_GRABBING3);
                    }else if(firstGrab && thirdGrab && allThreeSorted){
                        turret.stopOuttake();
                        turretLocalization.setPos(1);
                        follower.setMaxPower(0.6);
                        follower.followPath(Reverse3);
                        setPathState(PathState.GRABBING_REVERSAL3);
                        telemetry.addLine("DONE");
                    }
                }
                break;
            case BALLROW_GRABBING1:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1.5){
                    balls.setCurrent(new int[]{0,1,1});
                    if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
                        sorted = balls.sortBalls();
                    }
                    follower.setMaxPower(0.7);
                    follower.followPath(LeverPush2, true);
                    setPathState(PathState.LEVER2);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    balls.setCurrent(new int[]{1,0,1});
                    follower.setMaxPower(0.7);
                    follower.followPath(LeverPush, true);
                    setPathState(PathState.LEVER);
                    telemetry.addLine("Done Grabbing");
                    limelight.setPipeline(0);
                    turretLocalization.setPos(2);
                }
                break;
            case LEVER:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2.5){
                    scan();
                    if(limelight.getDetectedTagId() > 20) {
                        scanned = true;
                    }
                    if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
                        sorted = balls.sortBalls();
                    }
                    follower.setMaxPower(1.0);
                    turret.startOuttake();
                    follower.followPath(GrabbingReversal2, true);
                    setPathState(PathState.SHOOTING);
                    secondGrab = true;
                    allThreeSorted = false;
                    sortingIndex = 0;
                    pos = Shooting.MOVE;
                    stopAutonIntake();
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case LEVER2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2.5){
                    follower.setMaxPower(1.0);
                    turret.startOuttake();
                    firstGrab = true;
                    allThreeSorted = false;
                    sortingIndex = 0;
                    pos = Shooting.MOVE;
                    follower.followPath(GrabbingReversal1, true);
                    setPathState(PathState.SHOOTING);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING3:
                if(pathTimer.getElapsedTimeSeconds()>0.67) {
                    follower.setMaxPower(0.58);
                }
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1.0){
                    balls.setCurrent(new int[]{1,1,0});
                    if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
                        sorted = balls.sortBalls();
                    }
                    follower.setMaxPower(1.0);
                    turret.startOuttake();
                    follower.followPath(GrabbingReversal3, true);
                    thirdGrab = true;
                    allThreeSorted = false;
                    sortingIndex = 0;
                    pos = Shooting.MOVE;
                    setPathState(PathState.SHOOTING);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case GRABBING_REVERSAL3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>5){

                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        super.init();
        pathState = PathState.STARTPOS;
        //Add other init mechanisms
        follower.setPose(startPose);
    }
    @Override
    public void start(){
        super.start();
        setPathState(PathState.STARTPOS);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("Pos State", pos.toString());
        telemetry.addData("allThreeSorted", allThreeSorted);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Motif: ", Arrays.toString(balls.getFullMotif()));
        telemetry.addData("Current Balls ", Arrays.toString(balls.getCurrentBalls()));
        telemetry.addData("Sorted: ", Arrays.toString(sorted));
        telemetry.addData("Green in right spot", greenCheck);
        telemetry.addData("Pushed", pushed);
        telemetry.addData("Zero Grab", zeroGrab);
        telemetry.addData("allThreeunsorted", allThreeUnsorted);
        telemetry.addData("Servo Voltage: ", turretLocalization.getServoFeedback());
        telemetry.update();
    }
}

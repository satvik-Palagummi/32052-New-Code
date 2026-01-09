package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class FirstAuton extends AutonTemplate {
    public enum PathState {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        SHOOTPOS_FIRSTBALLROW,
        FIRSTBALLROW_GRABBING,
        GRABBING_REVERSAL,
    }
    PathState pathState;

    private final Pose startPose = new Pose(21,123, Math.toRadians(220));
    private final Pose shootPose = new Pose(50,93.5, Math.toRadians(220));
    private final Pose BallsRowAiming1 = new Pose(50,83.5, Math.toRadians(180));
    private final Pose grabFirstBalls = new Pose(20,83.5, Math.toRadians(180));
    private PathChain StartToShoot, shootToBallAiming1, AimingtoGrabbing1, GrabbingReversal1, ReversaltoAiming1;


    public void buildPaths(){
        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToBallAiming1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, BallsRowAiming1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), BallsRowAiming1.getHeading())
                .build();
        AimingtoGrabbing1 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming1, grabFirstBalls))
                .setLinearHeadingInterpolation(BallsRowAiming1.getHeading(), grabFirstBalls.getHeading())
                .build();
        GrabbingReversal1 = follower.pathBuilder()
                .addPath(new BezierLine(grabFirstBalls, BallsRowAiming1))
                .setLinearHeadingInterpolation(grabFirstBalls.getHeading(), BallsRowAiming1.getHeading())
                .build();
        ReversaltoAiming1 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming1, shootPose))
                .setLinearHeadingInterpolation(BallsRowAiming1.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        boolean firstGrab = false;
        boolean secondGrab = false;
        boolean thirdGrab = false;
        switch(pathState){
            case STARTPOS_SHOOTPOS:
                follower.followPath(StartToShoot, true);
                setPathState(PathState.SHOOT_PRELOAD);//Resets timer & makes new state
                break;
            case SHOOT_PRELOAD:
                //add logic to turret
                //check if follower is down with it's path.
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>5){
                    autonShoot();
                    if(!firstGrab) {
                        follower.followPath(shootToBallAiming1, true);
                        setPathState(PathState.SHOOTPOS_FIRSTBALLROW);
                        telemetry.addLine("Done Path 1");
                    }else{
                        telemetry.addLine("DONE");
                    }
                }
                break;
            case SHOOTPOS_FIRSTBALLROW:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(AimingtoGrabbing1, true);
                    setPathState(PathState.FIRSTBALLROW_GRABBING);
                    telemetry.addLine("Done Aiming towards Grab");
                    runAutonIntake();
                }
                break;
            case FIRSTBALLROW_GRABBING:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    stopAutonIntake();
                    follower.followPath(GrabbingReversal1, true);
                    setPathState(PathState.GRABBING_REVERSAL);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case GRABBING_REVERSAL:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(ReversaltoAiming1, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                    telemetry.addLine("Going to Shoot Position");
                    firstGrab = true;
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathState = PathState.STARTPOS_SHOOTPOS;;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //Add other init mechanisms
        buildPaths();
        follower.setPose(startPose);
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();

    }
}

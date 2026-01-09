package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
        STARTPOS,
        STARTPOS_SHOOTPOS,
        SHOOT1_SHOOT2,
        SHOOT2_SHOOT3,
        SHOOT_PRELOAD,
        PRELOAD_FIRSTBALLROW,
        FIRSTBALLROW_GRABBING,
        GRABBING_REVERSAL,
    }
    PathState pathState;

    private final Pose startPose = new Pose(21,123, Math.toRadians(220));

    private final Pose shootPosePos1 = new Pose(50,93.5, Math.toRadians(218));
    private final Pose shootPose = new Pose(50,93.5, Math.toRadians(218));
    private final Pose shootPosePos2 = new Pose(50,93.5, Math.toRadians(222));
    private final Pose shootPosePos3 = new Pose(50,93.5, Math.toRadians(227));
    private final Pose BallsRowAiming1 = new Pose(50,83.5, Math.toRadians(180));
    private final Pose grabFirstBalls = new Pose(20,83.5, Math.toRadians(180));
    private PathChain StartToShoot, ShootPose, ShootPos1To2,ShootPos2To3, shootToBallAiming1, AimingtoGrabbing1, GrabbingReversal1, ReversaltoAiming1;


    public void buildPaths(){
        follower.setStartingPose(startPose);
        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPosePos1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPosePos1.getHeading())
                .build();
        ShootPos1To2 = follower.pathBuilder()
                .setLinearHeadingInterpolation(shootPosePos1.getHeading(), shootPosePos2.getHeading())
                .build();
        ShootPos2To3 = follower.pathBuilder()
                .setLinearHeadingInterpolation(shootPosePos2.getHeading(), shootPosePos3.getHeading())
                .build();
        shootToBallAiming1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPosePos3, BallsRowAiming1))
                .setLinearHeadingInterpolation(shootPosePos3.getHeading(), BallsRowAiming1.getHeading())
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
                .addPath(new BezierLine(BallsRowAiming1, shootPosePos1))
                .setLinearHeadingInterpolation(BallsRowAiming1.getHeading(), shootPosePos1.getHeading())
                .build();
    }

    public void statePathUpdate(){
        boolean firstGrab = false;
        boolean secondGrab = false;
        boolean thirdGrab = false;
        switch(pathState){
            case STARTPOS:
                follower.followPath(StartToShoot, true);
                setPathState(PathState.STARTPOS_SHOOTPOS);//Resets timer & makes new state
                break;
            case STARTPOS_SHOOTPOS:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3)
                {
                    autonShoot(0);
                    follower.followPath(ShootPos1To2, true);
                    setPathState(PathState.SHOOT2_SHOOT3);
                }
            case SHOOT1_SHOOT2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3)
                {
                    autonShoot(1);
                    follower.followPath(ShootPos2To3, true);
                    setPathState(PathState.SHOOT2_SHOOT3);
                }
            case SHOOT2_SHOOT3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    autonShoot(2);
                    follower.followPath(ShootPos2To3,true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
            case SHOOT_PRELOAD:
                //add logic to turret
                //check if follower is down with it's path.
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>5){
                    //autonShoot();
                    if(!firstGrab) {
                        follower.followPath(shootToBallAiming1, true);
                        setPathState(PathState.PRELOAD_FIRSTBALLROW);
                        telemetry.addLine("Done Path 1");
                    }else{
                        telemetry.addLine("DONE");
                    }
                }
                break;
            case PRELOAD_FIRSTBALLROW:
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
        pathState = PathState.STARTPOS;;
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

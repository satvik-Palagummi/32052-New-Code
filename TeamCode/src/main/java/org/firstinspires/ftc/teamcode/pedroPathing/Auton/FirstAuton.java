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
        GRABBING_SHOOTPOS
    }
    PathState pathState;

    private final Pose startPose = new Pose(21,123, Math.toRadians(137));
    private final Pose shootPose = new Pose(50,93, Math.toRadians(137));
    private final Pose firstRowBallsAiming = new Pose(50,84, Math.toRadians(180));
    private final Pose grabFirstBalls = new Pose(19,84, Math.toRadians(180));
    private PathChain StartToShoot, shootToFirstBallAiming, FirstBallAimingtoGrabbing, GrabbingtoShoot;


    public void buildPaths(){
        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToFirstBallAiming = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowBallsAiming))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstRowBallsAiming.getHeading())
                .build();
        FirstBallAimingtoGrabbing = follower.pathBuilder()
                .addPath(new BezierLine(firstRowBallsAiming, grabFirstBalls))
                .setLinearHeadingInterpolation(firstRowBallsAiming.getHeading(), grabFirstBalls.getHeading())
                .build();
        GrabbingtoShoot = follower.pathBuilder()
                .addPath(new BezierLine(grabFirstBalls, shootPose))
                .setLinearHeadingInterpolation(grabFirstBalls.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
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
                    follower.followPath(shootToFirstBallAiming,true);
                    setPathState(PathState.SHOOTPOS_FIRSTBALLROW);
                    telemetry.addLine("Done Path 1");
                }
                break;
            case SHOOTPOS_FIRSTBALLROW:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(FirstBallAimingtoGrabbing, true);
                    setPathState(PathState.FIRSTBALLROW_GRABBING);
                    telemetry.addLine("Done Aiming towards Grab");
                }
                runAutonIntake();
                break;
            case FIRSTBALLROW_GRABBING:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){

                    follower.followPath(GrabbingtoShoot, true);
                    setPathState(PathState.GRABBING_SHOOTPOS);
                    telemetry.addLine("Done Grabbing");
                }
                stopAutonIntake();
                break;
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

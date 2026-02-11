package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class AutonFarBlue6 extends AutonTemplate {
    public enum PathState {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS,
        STARTPOS_SCANPOS,
        SCANPOS_SHOOTPOS,
        SHOOT_PRELOAD3, BALLROW_GRABBING3, GRABBING_REVERSAL3
    }
    PathState pathState;


    private final Pose startPose = new Pose(60,8, Math.toRadians(90));
    private final Pose scanPose = new Pose(60,20, Math.toRadians(-7));

    private final Pose shootPose = new Pose(60,10, Math.toRadians(20));
    private final Pose BallsRowAiming3 = new Pose(60, 35.5, Math.toRadians(0));
    private final Pose grabBalls3 = new Pose(15, 35.5, Math.toRadians(0));
    private PathChain StartToScan, ScantoShoot, shootToBallAiming3, AimingtoGrabbing3, GrabbingReversal3, ReversaltoAiming3;


    public void buildPaths(){
        StartToScan = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .build();
        ScantoShoot = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
        shootToBallAiming3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, BallsRowAiming3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), BallsRowAiming3.getHeading())
                .build();
        AimingtoGrabbing3 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming3, grabBalls3))
                .setLinearHeadingInterpolation(BallsRowAiming3.getHeading(), grabBalls3.getHeading())
                .build();
        GrabbingReversal3 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls3, BallsRowAiming3))
                .setLinearHeadingInterpolation(grabBalls3.getHeading(), BallsRowAiming3.getHeading())
                .build();
        ReversaltoAiming3 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming3, shootPose))
                .setLinearHeadingInterpolation(BallsRowAiming3.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){

        switch(pathState){
            case STARTPOS:
                turretLocalization.setPos(0);
                follower.followPath(StartToScan, true);
                setPathState(PathState.STARTPOS_SCANPOS);//Resets timer & makes new state
                break;
            case STARTPOS_SCANPOS:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3&& !scanned){
                    scan();
                    if(limelight.getDetectedTagId() > 20) {
                        scanned = true;
                    }

                }
                if(!follower.isBusy()&&scanned) {
                    follower.followPath(ScantoShoot, true);
                    setPathState(PathState.SCANPOS_SHOOTPOS);
                }
                break;
            case SCANPOS_SHOOTPOS:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>23)
                {
                    turret.setPower(1720);
                    turret.startOuttake();
                    if(!Arrays.equals(balls.getFullMotif(), new int[]{-1, -1, -1})) {
                        autonShoot3_5();
                    }else{
                        autonShoot2();
                    }
                    turret.stopOuttake();
                }
                break;
            case SHOOT_PRELOAD3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    follower.setMaxPower(0.4);
                    runAutonIntake();
                    follower.followPath(AimingtoGrabbing3,true);
                    setPathState(PathState.BALLROW_GRABBING3);
                    telemetry.addLine("Done Aiming towards Grab 3");
                }
                break;
            case BALLROW_GRABBING3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    balls.setCurrent(new int[]{0,1,1});
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(GrabbingReversal3, true);
                    setPathState(PathState.GRABBING_REVERSAL3);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case GRABBING_REVERSAL3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>0.5){
                    telemetry.addLine("Done");
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
        pathState = PathState.STARTPOS;;
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();

    }
}

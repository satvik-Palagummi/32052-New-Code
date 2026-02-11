package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class AutonFarBlue extends AutonTemplate {
    public enum PathState {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS,
        STARTPOS_SCANPOS,
        SCANPOS_SHOOTPOS,
        SHOOT1_SHOOT2,
        SHOOT2_SHOOT3,
        SHOOT_PRELOAD,
        BALLROW_GRABBING1,
        GRABBING_REVERSAL1,
    }
    PathState pathState;


    private final Pose startPose = new Pose(60,8, Math.toRadians(90));
    private final Pose scanPose = new Pose(60,20, Math.toRadians(-6));

    private final Pose shootPose = new Pose(60,10, Math.toRadians(20));
    private PathChain StartToScan, ScantoShoot;


    public void buildPaths(){
        StartToScan = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .build();
        ScantoShoot = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
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
                    //scan();
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
                    if(!Arrays.equals(balls.getFullMotif(), new int[]{-1, -1, -1})) {
                        autonShoot3_5();
                    }else{
                        autonShoot2();
                    }
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

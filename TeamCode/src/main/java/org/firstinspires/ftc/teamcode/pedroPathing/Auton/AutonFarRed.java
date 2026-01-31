package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonFarRed extends AutonTemplate {
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

    private final Pose startPose = new Pose(84,8, Math.toRadians(-90));
    private final Pose scanPose = new Pose(84,10, Math.toRadians(-19));

    private final Pose shootPose = new Pose(84,10, Math.toRadians(10));
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
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3)
                {
                    turret.setPower(1680);
                    turret.startOuttake();
                    if(balls.getFullMotif() !=null) {
                        autonShoot3();
                    }else{
                        autonShoot2_5();
                    }
                    turret.stopOuttake();
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

package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class AutonFarRed extends AutonTemplate {
    public enum PathState {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS,
        STARTPOS_SCANPOS,
        SCANPOS_SHOOTPOS,
        SHOOT_PRELOAD3, AIMING_GRABBING, GRABBING_REVERSAL
    }
    PathState pathState;

    private final Pose startPose = new Pose(84,9, Math.toRadians(-90));

    private final Pose shootPose = new Pose(84,11, Math.toRadians(-19));
    private final Pose GoGrabBalls = new Pose(125, 8, Math.toRadians(0));
    private PathChain StartToScan, StarttoShoot, shootToBallAiming3, AimingtoGrabbing, GrabbingReversal, ReversaltoAiming3;


    public void buildPaths(){
        StarttoShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
        AimingtoGrabbing = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, GoGrabBalls))
                .setLinearHeadingInterpolation(shootPose.getHeading(), GoGrabBalls.getHeading())
                .build();
        GrabbingReversal = follower.pathBuilder()
                .addPath(new BezierLine(GoGrabBalls, shootPose))
                .setLinearHeadingInterpolation(GoGrabBalls.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){

        switch(pathState){
            case STARTPOS:
                turretLocalization.setPos(0);
                follower.followPath(StartToScan, true);
                setPathState(PathState.SCANPOS_SHOOTPOS);//Resets timer & makes new state
                hoodMovement.setHood(0.3);
                turret.startOuttake();
                turret.setPower(1930);
                allThreeSorted = false;
                count = 0;
                balls.setCurrent(new int[]{1,1,0});
                break;
                /*
            case STARTPOS_SCANPOS:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3&& !scanned){
                    scan();
                    if(limelight.getDetectedTagId() > 20) {
                        scanned = true;
                    }

                }
                if(!follower.isBusy()&&scanned) {
                    follower.followPath(StarttoShoot, true);
                    setPathState(PathState.SCANPOS_SHOOTPOS);
                    sorted = balls.sortBalls();
                }
                break;

                 */
            case SCANPOS_SHOOTPOS:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3)
                {
                    stopAutonIntake();
                    autonShoot3Red();
                    if(allThreeSorted){
                        follower.setMaxPower(0.7);
                        runAutonIntake();
                        if(count<6) {
                            follower.followPath(AimingtoGrabbing, true);
                            setPathState(PathState.AIMING_GRABBING);
                        }
                    }
                }
                break;
            case AIMING_GRABBING:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    balls.setCurrent(new int[]{1,1,0});
                    count++;
                    allThreeSorted = false;
                    sortingIndex = 0;
                    pos = Shooting.MOVE;
                    reverseRed();
                    follower.followPath(GrabbingReversal);
                    setPathState(PathState.SCANPOS_SHOOTPOS);
                }
                break;
                /*
            case GRABBING_REVERSAL:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    balls.setCurrent(new int[]{0,1,1});
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(GrabbingReversal3, true);
                    setPathState(PathState.GRABBING_REVERSAL3);
                    telemetry.addLine("Done Grabbing");
                }
                break;

                 */
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

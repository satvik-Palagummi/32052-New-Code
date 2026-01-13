package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonFarBlue extends AutonTemplate {
    public enum PathState {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS,
        STARTPOS_SHOOTPOS,
        SHOOT1_SHOOT2,
        SHOOT2_SHOOT3,
        SHOOT_PRELOAD,
        BALLROW_GRABBING1,
        GRABBING_REVERSAL1,
    }
    PathState pathState;

    private final Pose startPose = new Pose(84,8, Math.toRadians(0));

    private final Pose shootPose = new Pose(84,10, Math.toRadians(19));
    private final Pose BallsRowAiming1 = new Pose(50,83.5, Math.toRadians(0));
    private final Pose grabFirstBalls = new Pose(16,83.5, Math.toRadians(0));
    private boolean firstGrab = false;
    private boolean secondGrab = false;
    private boolean thirdGrab = false;
    private PathChain StartToShoot, ShootPose, ShootPos1To2,ShootPos2To3, shootToBallAiming1, AimingtoGrabbing1, GrabbingReversal1, ReversaltoAiming1;


    public void buildPaths(){
        /*
        follower.setStartingPose(startPose);
        StartToShoot = new Path(new BezierLine(startPose, shootPosePos1));
        StartToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPosePos1.getHeading());
        ShootPos1To2 = new Path(new BezierLine(shootPosePos1, shootPosePos2));
        ShootPos1To2.setLinearHeadingInterpolation(shootPosePos1.getHeading(), shootPosePos2.getHeading());
        ShootPos2To3 = new Path(new BezierLine(shootPosePos2,shootPosePos3));
        ShootPos2To3.setLinearHeadingInterpolation(shootPosePos2.getHeading(),shootPosePos3.getHeading());
        shootToBallAiming1 = new Path(new BezierLine(shootPosePos3, BallsRowAiming1));
        shootToBallAiming1.setLinearHeadingInterpolation(shootPosePos3.getHeading(), BallsRowAiming1.getHeading());
        AimingtoGrabbing1 = new Path(new BezierLine(grabFirstBalls,BallsRowAiming1));
        AimingtoGrabbing1.setLinearHeadingInterpolation(BallsRowAiming1.getHeading(), grabFirstBalls.getHeading());
        GrabbingReversal1 = new Path(new BezierLine(grabFirstBalls, BallsRowAiming1));
        GrabbingReversal1.setLinearHeadingInterpolation(grabFirstBalls.getHeading(), BallsRowAiming1.getHeading());
        ReversaltoAiming1 = new Path(new BezierLine(BallsRowAiming1, shootPosePos1));
        ReversaltoAiming1.setLinearHeadingInterpolation(BallsRowAiming1.getHeading(), shootPosePos1.getHeading());
         */
        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        /*ShootPos1To2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPosePos1, shootPosePos2))
                .setLinearHeadingInterpolation(shootPosePos1.getHeading(), shootPosePos2.getHeading())
                .build();
        ShootPos2To3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPosePos2, shootPosePos3))
                .setLinearHeadingInterpolation(shootPosePos2.getHeading(), shootPosePos3.getHeading())
                .build();


        shootToBallAiming1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, BallsRowAiming1))
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
        */
    }

    public void statePathUpdate(){

        switch(pathState){
            case STARTPOS:
                follower.followPath(StartToShoot, true);
                setPathState(PathState.STARTPOS_SHOOTPOS);//Resets timer & makes new state
                break;
            case STARTPOS_SHOOTPOS:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3)
                {
                    autonShoot2_5();
                    if(!firstGrab) {
                        follower.followPath(shootToBallAiming1, true);
                        setPathState(PathState.SHOOT_PRELOAD);
                    }else{
                        telemetry.addLine("DONE");
                    }
                }
                break;
                /*
            case SHOOT1_SHOOT2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3)
                {
                    autonShoot(1);
                    follower.followPath(ShootPos2To3, true);
                    setPathState(PathState.SHOOT2_SHOOT3);
                }
                break;
            case SHOOT2_SHOOT3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    autonShoot(2);
                    follower.followPath(ShootPos2To3,true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                //add logic to turret
                //check if follower is down with it's path.
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>5){
                    //autonShoot();
                    follower.followPath(AimingtoGrabbing1, true);
                    setPathState(PathState.BALLROW_GRABBING1);
                    telemetry.addLine("Done Aiming towards Grab");
                }
                break;
            case BALLROW_GRABBING1:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    stopAutonIntake();
                    follower.followPath(GrabbingReversal1, true);
                    setPathState(PathState.GRABBING_REVERSAL1);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case GRABBING_REVERSAL1:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(ReversaltoAiming1, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                    telemetry.addLine("Going to Shoot Position");
                    firstGrab = true;
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

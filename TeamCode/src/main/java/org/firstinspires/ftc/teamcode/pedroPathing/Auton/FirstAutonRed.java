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
public class FirstAutonRed extends AutonTemplate {
    public enum PathState {
        //START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        STARTPOS,
        SCANPOSE,
        SHOOTING,
        SHOOT1_SHOOT2,
        SHOOT2_SHOOT3,
        SHOOT_PRELOAD1, BALLROW_GRABBING1, GRABBING_REVERSAL1,
        SHOOT_PRELOAD2, BALLROW_GRABBING2, GRABBING_REVERSAL2,
        SHOOT_PRELOAD3, BALLROW_GRABBING3, GRABBING_REVERSAL3
    }
    PathState pathState;

    private final Pose startPose = new Pose(124,125, Math.toRadians(-54));
    private final Pose scanPose = new Pose(90, 121, Math.toRadians(30));
    private final Pose shootPose = new Pose(83,83.5, Math.toRadians(-43));
    private final Pose BallsRowAiming1 = new Pose(100,83.5, Math.toRadians(0));
    private final Pose grabBalls1 = new Pose(130,83.5, Math.toRadians(0));
    private final Pose BallsRowAiming2 = new Pose(83, 60,Math.toRadians(0));
    private final Pose grabBalls2 = new Pose(135, 60, Math.toRadians(0));
    private final Pose BallsRowAiming3 = new Pose(83, 35.5, Math.toRadians(0));
    private final Pose grabBalls3 = new Pose(135, 35.5, Math.toRadians(0));
    private boolean firstGrab = false;
    private boolean secondGrab = false;
    private boolean thirdGrab = false;
    private PathChain StartToScan,
            ShootPose,
            ShootPos1To2,
            ShootPos2To3,
            shootToBallAiming1, AimingtoGrabbing1, GrabbingReversal1, ReversaltoAiming1,
            shootToBallAiming2, AimingtoGrabbing2, GrabbingReversal2, ReversaltoAiming2,
            shootToBallAiming3, AimingtoGrabbing3, GrabbingReversal3, ReversaltoAiming3;


    public void buildPaths(){
        StartToScan = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .build();
        ShootPose = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, shootPose))
                .setLinearHeadingInterpolation(scanPose.getHeading(), shootPose.getHeading())
                .build();
        /*ShootPos1To2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPosePos1, shootPosePos2))
                .setLinearHeadingInterpolation(shootPosePos1.getHeading(), shootPosePos2.getHeading())
                .build();
        ShootPos2To3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPosePos2, shootPosePos3))
                .setLinearHeadingInterpolation(shootPosePos2.getHeading(), shootPosePos3.getHeading())
                .build();

         */
        //FIRST ROW PATHS

        shootToBallAiming1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, BallsRowAiming1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), BallsRowAiming1.getHeading())
                .build();
        AimingtoGrabbing1 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming1, grabBalls1))
                .setLinearHeadingInterpolation(BallsRowAiming1.getHeading(), grabBalls1.getHeading())
                .build();
        GrabbingReversal1 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls1, BallsRowAiming1))
                .setLinearHeadingInterpolation(grabBalls1.getHeading(), BallsRowAiming1.getHeading())
                .build();
        ReversaltoAiming1 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming1, shootPose))
                .setLinearHeadingInterpolation(BallsRowAiming1.getHeading(), shootPose.getHeading())
                .build();
        //SECOND ROW PATHS

        shootToBallAiming2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, BallsRowAiming2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), BallsRowAiming2.getHeading())
                .build();
        AimingtoGrabbing2 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming2, grabBalls2))
                .setLinearHeadingInterpolation(BallsRowAiming2.getHeading(), grabBalls2.getHeading())
                .build();
        GrabbingReversal2 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls2, BallsRowAiming1))
                .setLinearHeadingInterpolation(grabBalls2.getHeading(), BallsRowAiming2.getHeading())
                .build();
        ReversaltoAiming2 = follower.pathBuilder()
                .addPath(new BezierLine(BallsRowAiming2, shootPose))
                .setLinearHeadingInterpolation(BallsRowAiming2.getHeading(), shootPose.getHeading())
                .build();
        //THIRD ROW PATHS

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
                balls.setCurrent(1);
                if(!checkGreen()){
                    balls.setCurrent(0);
                }
                follower.followPath(StartToScan, true);
                setPathState(PathState.SCANPOSE);//Resets timer & makes new state
                break;
            case SCANPOSE:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>10) {
                    scan();
                    follower.followPath(ShootPose, true);
                    setPathState(PathState.SHOOTING);
                }
                break;
            case SHOOTING:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3)
                {
                    autonShoot3();
                    if(!firstGrab) {
                        follower.followPath(shootToBallAiming1, true);
                        setPathState(PathState.SHOOT_PRELOAD1);
                    }else if(!secondGrab){
                        follower.followPath(shootToBallAiming2, true);
                        setPathState(PathState.SHOOT_PRELOAD2);
                    } else if (!thirdGrab) {
                        follower.followPath(shootToBallAiming3, true);
                        setPathState(PathState.SHOOT_PRELOAD3);
                    }
                    else{
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
                */
            case SHOOT_PRELOAD1:
                //add logic to turret
                //check if follower is down with it's path.
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    runAutonIntake();
                    follower.followPath(AimingtoGrabbing1, true);
                    setPathState(PathState.BALLROW_GRABBING1);
                    telemetry.addLine("Done Aiming towards Grab 1");
                }
                break;
            case SHOOT_PRELOAD2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>4){
                    runAutonIntake();
                    follower.followPath(AimingtoGrabbing2, true);
                    setPathState(PathState.BALLROW_GRABBING2);
                    telemetry.addLine("Done Aiming towards Grab 2");
                }
                break;
            case SHOOT_PRELOAD3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>4){
                    runAutonIntake();
                    follower.followPath(AimingtoGrabbing3,true);
                    setPathState(PathState.BALLROW_GRABBING3);
                    telemetry.addLine("Done Aiming towards Grab 3");
                }
                break;
            case BALLROW_GRABBING1:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    stopAutonIntake();
                    balls.setCurrent(1);
                    if(!checkGreen()){
                        balls.setCurrent(0);
                    }
                    follower.followPath(GrabbingReversal1, true);
                    setPathState(PathState.GRABBING_REVERSAL1);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    stopAutonIntake();
                    balls.setCurrent(2);
                    if(!checkGreen()){
                        balls.setCurrent(0);
                    }
                    follower.followPath(GrabbingReversal2, true);
                    setPathState(PathState.GRABBING_REVERSAL2);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    stopAutonIntake();
                    balls.setCurrent(3);
                    if(!checkGreen()){
                        balls.setCurrent(0);
                    }
                    follower.followPath(GrabbingReversal3, true);
                    setPathState(PathState.GRABBING_REVERSAL3);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case GRABBING_REVERSAL1:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(ReversaltoAiming1, true);
                    setPathState(PathState.SHOOTING);
                    telemetry.addLine("Going to Shoot Position");
                    firstGrab = true;
                }
                break;
            case GRABBING_REVERSAL2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(ReversaltoAiming2, true);
                    setPathState(PathState.SHOOTING);
                    telemetry.addLine("Going to Shoot Position");
                    secondGrab = true;
                }
                break;
            case GRABBING_REVERSAL3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3){
                    follower.followPath(ReversaltoAiming3, true);
                    setPathState(PathState.SHOOTING);
                    telemetry.addLine("Going to Shoot Position");
                    thirdGrab = true;
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
        telemetry.addData("Blue: ", colorsensor.getBlue());
        telemetry.addData("Green: ", colorsensor.getGreen());
        telemetry.addData("Motif: ", balls.getFullMotif());
        telemetry.addData("Current Balls ", balls.getCurrentBalls());
        telemetry.update();

    }
}

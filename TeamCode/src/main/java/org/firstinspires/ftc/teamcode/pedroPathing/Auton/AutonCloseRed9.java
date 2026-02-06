package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class AutonCloseRed9 extends AutonTemplate {
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
    private final Pose scanPose = new Pose(90, 121, Math.toRadians(35));
    private final Pose shootPose = new Pose(84,86, Math.toRadians(-50));
    private final Pose grabBalls1 = new Pose(125,83.5, Math.toRadians(0));
    private final Pose grabBalls1Control = new Pose(76, 90);
    private final Pose grabBalls2 = new Pose(130, 59, Math.toRadians(0));
    private final Pose grabBalls2Control = new Pose(70,62);
    private final Pose shootPos2Control = new Pose(80, 60);
    private final Pose grabBalls3 = new Pose(130, 35, Math.toRadians(5));
    private final Pose grabBalls3Control = new Pose(65, 37);
    private final Pose shootPos3Control = new Pose(81, 46);
    private boolean firstGrab = false;
    private boolean secondGrab = false;
    private boolean thirdGrab = false;
    private PathChain StartToScan,
            ShootPose,
            ShootPos1To2,
            ShootPos2To3,
            shootToBallGrabbing1, GrabbingReversal1,
            shootToBallGrabbing2, GrabbingReversal2, ReversaltoAiming2,
            shootToBallGrabbing3, GrabbingReversal3, ReversaltoAiming3;


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

        shootToBallGrabbing1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls1Control, grabBalls1))
                .setConstantHeadingInterpolation(grabBalls1.getHeading())
                .build();

        GrabbingReversal1 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls1, shootPose))
                .setLinearHeadingInterpolation(grabBalls1.getHeading(), grabBalls1.getHeading())
                .build();
        //SECOND ROW PATHS

        shootToBallGrabbing2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls2Control, grabBalls2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), grabBalls2.getHeading())
                .build();

        GrabbingReversal2 = follower.pathBuilder()
                .addPath(new BezierCurve(grabBalls2, shootPos2Control, shootPose))
                .setLinearHeadingInterpolation(grabBalls2.getHeading(), shootPose.getHeading())
                .build();
        //THIRD ROW PATHS

        shootToBallGrabbing3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls3Control, grabBalls3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), grabBalls3.getHeading())
                .build();
        GrabbingReversal3 = follower.pathBuilder()
                .addPath(new BezierCurve(grabBalls3, shootPos3Control, shootPose))
                .setLinearHeadingInterpolation(grabBalls3.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case STARTPOS:
                balls.setCurrent(new int[]{1,1,0});
                turretLocalization.setPos(2);
                limelight.setPipeline(0);
                follower.followPath(StartToScan, true);
                setPathState(PathState.SCANPOSE);//Resets timer & makes new state
                break;
            case SCANPOSE:
                if(!follower.isBusy()&& !scanned){
                    scan();
                    turret.setPower(1365);
                    if(limelight.getDetectedTagId() > 20) {
                        scanned = true;
                    }
                }
                if(!follower.isBusy()&&scanned) {
                    follower.followPath(ShootPose, true);
                    setPathState(PathState.SHOOTING);
                }

                break;
            case SHOOTING:
                if(!follower.isBusy())
                {
                    stopAutonIntake();
                    turret.startOuttake();
                    autonShoot3();

                    turret.stopOuttake();
                    if(!firstGrab) {
                        follower.setMaxPower(0.45);
                        runAutonIntake();
                        follower.followPath(shootToBallGrabbing1, true);
                        setPathState(PathState.BALLROW_GRABBING1);
                    }else if(!secondGrab) {
                        follower.setMaxPower(0.5);
                        runAutonIntake();
                        follower.followPath(shootToBallGrabbing2, true);
                        setPathState(PathState.BALLROW_GRABBING2);
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
                */
            case BALLROW_GRABBING1:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1.5){
                    balls.setCurrent(new int[]{1,1,0});
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(GrabbingReversal1, true);
                    setPathState(PathState.SHOOTING);
                    firstGrab = true;
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    balls.setCurrent(new int[]{1,0,1});
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(GrabbingReversal2, true);
                    setPathState(PathState.SHOOTING);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING3:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    balls.setCurrent(new int[]{0,1,1});
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(GrabbingReversal3, true);
                    setPathState(PathState.SHOOTING);
                    telemetry.addLine("Done Grabbing");
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
        telemetry.addData("Motif: ", Arrays.toString(balls.getFullMotif()));
        telemetry.addData("Current Balls ", Arrays.toString(balls.getCurrentBalls()));
        telemetry.addData("Green in right spot", greenCheck);
        telemetry.addData("Pushed", pushed);
        telemetry.update();

    }
}

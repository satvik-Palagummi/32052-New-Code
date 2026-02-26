package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

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
        SHOOT_PRELOAD2, BALLROW_GRABBING2, LEVER,
        SHOOT_PRELOAD3, BALLROW_GRABBING3, GRABBING3SLOW
    }
    PathState pathState;

    private final Pose startPose = new Pose(120.5,126, Math.toRadians(36));
    private final Pose scanPose = new Pose(93, 94, Math.toRadians(25));
    private final Pose scanningControl = new Pose(61,63);
    private final Pose ZeroGrabShoot = new Pose(84,83, Math.toRadians(-48));
    private final Pose shootPose = new Pose(84,83, Math.toRadians(-45));
    private final Pose grabBalls1 = new Pose(126,82, Math.toRadians(0));
    private final Pose grabBalls1Control = new Pose(76, 91.5);
    private final Pose grabBalls2 = new Pose(133, 58.5, Math.toRadians(0));
    private final Pose grabBalls2Control = new Pose(67,63);
    private final Pose hitLever = new Pose(128,69,Math.toRadians(-100));
    private final Pose hitLeverControl = new Pose(82, 60);
    private final Pose shootPos2Control = new Pose(80, 60);
    private final Pose grabBalls3 = new Pose(130, 36, Math.toRadians(-5));
    private final Pose shootPose3Orient = new Pose(94,70,Math.toRadians(-50));
    private final Pose grabBalls3Control = new Pose(67, 33);
    private final Pose shootPos3Control = new Pose(81, 46);
    private boolean zeroGrab = false;
    private boolean firstGrab = false;
    private boolean secondGrab = false;
    private boolean thirdGrab = false;
    private PathChain StartToShoot,
            ShootPose,
            ShootPos1To2,
            ShootPos2To3,
            shootToBallGrabbing1, GrabbingReversal1,
            shootToBallGrabbing2, GrabbingReversal2, LeverPush,
            shootToBallGrabbing3, GrabbingReversal3, Reverse3;


    public void buildPaths(){
        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .build();
        ShootPose = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, ZeroGrabShoot))
                .setLinearHeadingInterpolation(scanPose.getHeading(),ZeroGrabShoot.getHeading())
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
                .setLinearHeadingInterpolation(grabBalls1.getHeading(), shootPose.getHeading())
                .build();
        //SECOND ROW PATHS

        shootToBallGrabbing2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls2Control, grabBalls2))
                .setLinearHeadingInterpolation(ZeroGrabShoot.getHeading(), grabBalls2.getHeading())
                .build();
        LeverPush = follower.pathBuilder()
                .addPath(new BezierCurve(grabBalls2, hitLeverControl, hitLever))
                .setLinearHeadingInterpolation(grabBalls2.getHeading(), hitLever.getHeading())
                .build();
        GrabbingReversal2 = follower.pathBuilder()
                .addPath(new BezierCurve(hitLever, shootPos2Control, shootPose))
                .setLinearHeadingInterpolation(hitLever.getHeading(), shootPose.getHeading())
                .build();
        //THIRD ROW PATHS

        shootToBallGrabbing3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls3Control, grabBalls3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), grabBalls3.getHeading())
                .build();
        GrabbingReversal3 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls3, shootPose))
                .setLinearHeadingInterpolation(grabBalls3.getHeading(), shootPose.getHeading())
                .build();
        Reverse3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, shootPose3Orient))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case STARTPOS:
                balls.setCurrent(new int[]{1,1,0});
                turretLocalization.setPos(2);
                limelight.setPipeline(0);
                turret.setPower(1382);
                turret.startOuttake();
                follower.followPath(StartToShoot, true);
                setPathState(PathState.SCANPOSE);//Resets timer & makes new state
                break;
            case SCANPOSE:
                if(!follower.isBusy()) {
                    scan();
                    if(limelight.getDetectedTagId() > 20) {
                        scanned = true;
                    }
                    follower.followPath(ShootPose, true);
                    setPathState(PathState.SHOOTING);
                }


                break;
            case SHOOTING:
                if(!follower.isBusy())
                {
                    if(!zeroGrab){
                        autonShoot2_5Red();
                    }else{
                        stopAutonIntake();

                        autonShoot3Red();
                    }

                    if(allThreeUnsorted){
                        if(!zeroGrab){
                            zeroGrab = true;
                        }
                        if(!secondGrab) {
                            turretLocalization.setPos(1);
                            turret.stopOuttake();
                            follower.setMaxPower(0.55);
                            runAutonIntake();
                            follower.followPath(shootToBallGrabbing2, true);
                            setPathState(PathState.BALLROW_GRABBING2);
                        }else if(!firstGrab&&allThreeSorted){
                            turretLocalization.setPos(1);
                            turret.stopOuttake();
                            follower.setMaxPower(0.61);
                            runAutonIntake();
                            follower.followPath(shootToBallGrabbing1, true);
                            setPathState(PathState.BALLROW_GRABBING1);
                        } else if (!thirdGrab&&allThreeSorted) {
                            turretLocalization.setPos(1);
                            turret.stopOuttake();
                            follower.setMaxPower(0.9);
                            runAutonIntake();
                            follower.followPath(shootToBallGrabbing3, true);
                            setPathState(PathState.BALLROW_GRABBING3);
                        }else if(firstGrab && thirdGrab && allThreeSorted){
                            turretLocalization.setPos(1);
                            turret.stopOuttake();
                            follower.followPath(Reverse3);
                            setPathState(PathState.GRABBING3SLOW);
                            telemetry.addLine("DONE");
                        }
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
                    if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
                        sorted = balls.sortBalls();
                    }
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    turret.startOuttake();
                    follower.followPath(GrabbingReversal1, true);
                    setPathState(PathState.SHOOTING);
                    firstGrab = true;
                    allThreeSorted = false;
                    pos = Shooting.MOVE;
                    sortingIndex = 0;
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    balls.setCurrent(new int[]{1,0,1});
                    if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
                        sorted = balls.sortBalls();
                    }
                    follower.setMaxPower(0.8);
                    stopAutonIntake();
                    follower.followPath(LeverPush, true);
                    setPathState(PathState.LEVER);
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case LEVER:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2.5){
                    follower.setMaxPower(1.0);
                    turret.startOuttake();
                    follower.followPath(GrabbingReversal2, true);
                    setPathState(PathState.SHOOTING);
                    secondGrab = true;
                    allThreeSorted = false;
                    pos = Shooting.MOVE;
                    sortingIndex = 0;
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case BALLROW_GRABBING3:
                if(pathTimer.getElapsedTimeSeconds()>0.67) {
                    follower.setMaxPower(0.56);
                }
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1.0){
                    balls.setCurrent(new int[]{0,1,1});
                    if(balls.getFullMotif() != null && balls.getCurrentBalls() != null){
                        sorted = balls.sortBalls();
                    }
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    turret.startOuttake();
                    follower.followPath(GrabbingReversal3, true);
                    setPathState(PathState.SHOOTING);
                    thirdGrab = true;
                    allThreeSorted = false;
                    pos = Shooting.MOVE;
                    sortingIndex = 0;
                    telemetry.addLine("Done Grabbing");
                }
                break;
            case GRABBING3SLOW:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>5){
                }
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
        pathState = PathState.STARTPOS;
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
        telemetry.addData("pos state", pos.toString());
        telemetry.addData("allThreeSorted", allThreeSorted);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Motif: ", Arrays.toString(balls.getFullMotif()));
        telemetry.addData("Current Balls ", Arrays.toString(balls.getCurrentBalls()));
        telemetry.addData("Sorted: ", Arrays.toString(sorted));
        telemetry.addData("Green in right spot", greenCheck);
        telemetry.addData("Pushed", pushed);
        telemetry.update();

    }
}

package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous
public class FirstAutonBlue extends AutonTemplate {
    public enum PathState {
        STARTPOS,
        SHOOT1,
        GRAB1,
        SHOOT2,
        REVERSAL1,
        GRAB2,
        PUSH2,
        SHOOT3,
        GRAB3,
        SHOOT4,
        PARKING1,
        PARKING2,
        FINAL_SHOOT,
        DONE
    }
    PathState pathState;

    private final Pose startPose = new Pose(32.700, 135.670, Math.toRadians(0));
    private final Pose shootPose = new Pose(60.000, 83.000, Math.toRadians(45));
    private final Pose grabBalls1 = new Pose(16.000, 83.000, Math.toRadians(0));
    private final Pose hitLeverFirstSpike = new Pose(13.984, 71.664, Math.toRadians(85));
    private final Pose hitLeverFirstSpikeControl = new Pose(31.222, 77.000);
    private final Pose grabBalls2 = new Pose(10.000, 60.000, Math.toRadians(0));
    private final Pose grabBalls2Control = new Pose(58.214, 56.951);
    private final Pose hitLever = new Pose(13.975, 71.328, Math.toRadians(-20));
    private final Pose hitLeverControl = new Pose(38.308, 62.016);
    private final Pose shootPos2Control = new Pose(54.254, 69.746);
    private final Pose grabBalls3 = new Pose(11.008, 35.832, Math.toRadians(0));
    private final Pose grabBalls3Control = new Pose(75.613, 31.925);
    
    // Updated Poses for Parking/End Sequence
    private final Pose parkTransitionPose = new Pose(60.000, 11.252, Math.toRadians(45));
    private final Pose parkPose2 = new Pose(9.914, 8.737, Math.toRadians(0));
    private final Pose parkPose3 = new Pose(59.930, 11.265, Math.toRadians(20));

    private boolean shootingStarted = false;

    private PathChain StartToShoot, shootToBallGrabbing1, LeverPush2, GrabbingReversal1,
            shootToBallGrabbing2, LeverPush, GrabbingReversal2,
            shootToBallGrabbing3, GrabbingReversal3,
            Park1, Park2;

    public void buildPaths() {
        // Path 1
        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Path 2
        shootToBallGrabbing1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, grabBalls1))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 3
        LeverPush2 = follower.pathBuilder()
                .addPath(new BezierCurve(grabBalls1, hitLeverFirstSpikeControl, hitLeverFirstSpike))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(85))
                .build();

        // Path 4
        GrabbingReversal1 = follower.pathBuilder()
                .addPath(new BezierLine(hitLeverFirstSpike, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(85), Math.toRadians(45))
                .build();

        // Path 5
        shootToBallGrabbing2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls2Control, grabBalls2))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 6
        LeverPush = follower.pathBuilder()
                .addPath(new BezierCurve(grabBalls2, hitLeverControl, hitLever))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        // Path 7
        GrabbingReversal2 = follower.pathBuilder()
                .addPath(new BezierCurve(hitLever, shootPos2Control, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(45))
                .build();

        // Path 8
        shootToBallGrabbing3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, grabBalls3Control, grabBalls3))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 9
        GrabbingReversal3 = follower.pathBuilder()
                .addPath(new BezierLine(grabBalls3, parkTransitionPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 10
        Park1 = follower.pathBuilder()
                .addPath(new BezierLine(parkTransitionPose, parkPose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 11
        Park2 = follower.pathBuilder()
                .addPath(new BezierLine(parkPose2, parkPose3))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case STARTPOS:
                // Functional initializations from older version
                hoodMovement.setHood(0.43);
                balls.setCurrent(new int[]{1, 1, 0});
                sorted = balls.sortBalls();
                turretLocalization.setPos(1);
                limelight.setPipeline(8);
                turret.setPower(1530);
                turret.startOuttake();
                allThreeSorted = false;
                
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(StartToShoot, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (!follower.isBusy()) {
                    if (!shootingStarted) {
                        autonShoot3Blue();
                        shootingStarted = true;
                    }
                    if (allThreeSorted) {
                        shootingStarted = false;
                        allThreeSorted = false;
                        follower.setMaxPower(0.8);
                        follower.followPath(shootToBallGrabbing1, true);
                        setPathState(PathState.GRAB1);
                    }
                }
                break;

            case GRAB1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    balls.setCurrent(new int[]{0, 1, 1});
                    sorted = balls.sortBalls();
                    allThreeSorted = false;
                    sortingIndex = 0;
                    pos = Shooting.MOVE;

                    runAutonIntake();
                    follower.setMaxPower(0.7);
                    follower.followPath(LeverPush2, true);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (!follower.isBusy()) {
                    stopAutonIntake();
                    if (!shootingStarted) {
                        autonShoot3Blue();
                        shootingStarted = true;
                    }
                    if (allThreeSorted) {
                        shootingStarted = false;
                        allThreeSorted = false;
                        follower.setMaxPower(1.0);
                        follower.followPath(GrabbingReversal1, true);
                        setPathState(PathState.REVERSAL1);
                    }
                }
                break;

            case REVERSAL1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(shootToBallGrabbing2, true);
                    setPathState(PathState.GRAB2);
                }
                break;

            case GRAB2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    balls.setCurrent(new int[]{1, 0, 1});
                    sorted = balls.sortBalls();
                    allThreeSorted = false;
                    sortingIndex = 0;
                    pos = Shooting.MOVE;
                    
                    runAutonIntake();
                    follower.setMaxPower(0.7);
                    follower.followPath(LeverPush, true);
                    setPathState(PathState.PUSH2);
                }
                break;

            case PUSH2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(GrabbingReversal2, true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!follower.isBusy()) {
                    stopAutonIntake();
                    if (!shootingStarted) {
                        autonShoot3Blue();
                        shootingStarted = true;
                    }
                    if (allThreeSorted) {
                        shootingStarted = false;
                        allThreeSorted = false;
                        follower.setMaxPower(1.0);
                        follower.followPath(shootToBallGrabbing3, true);
                        setPathState(PathState.GRAB3);
                    }
                }
                break;

            case GRAB3:
                if (pathTimer.getElapsedTimeSeconds() > 0.67) {
                    follower.setMaxPower(0.58);
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    balls.setCurrent(new int[]{1, 1, 0});
                    sorted = balls.sortBalls();
                    allThreeSorted = false;
                    sortingIndex = 0;
                    pos = Shooting.MOVE;

                    runAutonIntake();
                    follower.setMaxPower(1.0);
                    follower.followPath(GrabbingReversal3, true);
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if (!follower.isBusy()) {
                    stopAutonIntake();
                    if (!shootingStarted) {
                        autonShoot3Blue();
                        shootingStarted = true;
                    }
                    if (allThreeSorted) {
                        shootingStarted = false;
                        allThreeSorted = false;
                        follower.setMaxPower(0.6);
                        follower.followPath(Park1, true);
                        setPathState(PathState.PARKING1);
                    }
                }
                break;

            case PARKING1:
                if (!follower.isBusy()) {
                    follower.followPath(Park2, true);
                    setPathState(PathState.FINAL_SHOOT);
                }
                break;

            case FINAL_SHOOT:
                if (!follower.isBusy()) {
                    if (!shootingStarted) {
                        autonShoot3Blue();
                        shootingStarted = true;
                    }
                    if (allThreeSorted) {
                        shootingStarted = false;
                        allThreeSorted = false;
                        setPathState(PathState.DONE);
                    }
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous Complete");
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
        follower.setPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        super.start();
        setPathState(PathState.STARTPOS);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("Pos State", pos.toString());
        telemetry.addData("allThreeSorted", allThreeSorted);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Motif: ", Arrays.toString(balls.getFullMotif()));
        telemetry.addData("Current Balls ", Arrays.toString(balls.getCurrentBalls()));
        telemetry.addData("Sorted: ", Arrays.toString(sorted));
        telemetry.update();
    }
}

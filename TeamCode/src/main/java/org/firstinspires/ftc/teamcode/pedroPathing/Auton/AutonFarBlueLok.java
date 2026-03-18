package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonFarBlueLok", group = "Autonomous")
public class AutonFarBlueLok extends AutonTemplate {
    public enum PathState {
        PATH_1,
        PATH_2,
        PATH_3,
        SHOOT_1,
        PATH_4,
        PATH_5,
        SHOOT_2,
        PATH_6,
        PATH_7,
        SHOOT_3,
        PATH_8,
        PATH_9,
        SHOOT_4,
        END
    }

    private PathState pathState;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(15));

    @Override
    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(56.000, 8.000), new Pose(56.705, 37.239), new Pose(36.845, 35.664)))
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(36.845, 35.664), new Pose(7.057, 35.118)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(7.057, 35.118), new Pose(55.858, 8.252)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.858, 8.252), new Pose(8.735, 8.636)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8.735, 8.636), new Pose(55.708, 8.401)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.708, 8.401), new Pose(8.735, 8.636)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8.735, 8.636), new Pose(55.708, 8.401)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.708, 8.401), new Pose(8.735, 8.636)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8.735, 8.636), new Pose(55.708, 8.401)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();
    }

    @Override
    public void statePathUpdate() {
        switch (pathState) {
            case PATH_1:
                follower.followPath(path1, true);
                setPathState(PathState.PATH_2);
                break;
            case PATH_2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    runAutonIntake();
                    follower.followPath(path2, true);
                    setPathState(PathState.PATH_3);
                }
                break;
            case PATH_3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(path3, true);
                    setPathState(PathState.SHOOT_1);
                }
                break;
            case SHOOT_1:
                if (!follower.isBusy()) {
                    autonShoot2();
                    setPathState(PathState.PATH_4);
                }
                break;
            case PATH_4:
                follower.setMaxPower(0.7);
                runAutonIntake();
                follower.followPath(path4, true);
                setPathState(PathState.PATH_5);
                break;
            case PATH_5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(path5, true);
                    setPathState(PathState.SHOOT_2);
                }
                break;
            case SHOOT_2:
                if (!follower.isBusy()) {
                    autonShoot2();
                    setPathState(PathState.PATH_6);
                }
                break;
            case PATH_6:
                follower.setMaxPower(0.7);
                runAutonIntake();
                follower.followPath(path6, true);
                setPathState(PathState.PATH_7);
                break;
            case PATH_7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(path7, true);
                    setPathState(PathState.SHOOT_3);
                }
                break;
            case SHOOT_3:
                if (!follower.isBusy()) {
                    autonShoot2();
                    setPathState(PathState.PATH_8);
                }
                break;
            case PATH_8:
                follower.setMaxPower(0.7);
                runAutonIntake();
                follower.followPath(path8, true);
                setPathState(PathState.PATH_9);
                break;
            case PATH_9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    stopAutonIntake();
                    follower.followPath(path9, true);
                    setPathState(PathState.SHOOT_4);
                }
                break;
            case SHOOT_4:
                if (!follower.isBusy()) {
                    autonShoot2();
                    setPathState(PathState.END);
                }
                break;
            case END:
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
        pathState = PathState.PATH_1;
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        super.start();
        turret.startOuttake();
        turret.setPower(1930);
        hoodMovement.setHood(0.3);
        setPathState(PathState.PATH_1);
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}

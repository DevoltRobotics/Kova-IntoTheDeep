package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "PedroPathAuto", group = "PedroPath")
public class SpecimenAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(9, 56, Math.toRadians(0));
    private final Pose scorePose = new Pose(37, 69, Math.toRadians(0));
    private final Pose sample1 = new Pose(59, 23, Math.toRadians(180));
    private final Pose sample2 = new Pose(59, 14, Math.toRadians(180));
    private final Pose leaveSample = new Pose(17, 22, Math.toRadians(180));
    private final Pose waitforHuman = new Pose(28,22, Math.toRadians(180));

    private final Pose grabSpecimen = new Pose(10,22,Math.toRadians(0));


    private Path scorePreload, scoreToSample1, sample1ToHuman, grabToScoreSpecimen;
    private PathChain humanToSample2ToHuman, leaveToWaitToGrab;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        scoreToSample1 = new Path(new BezierCurve(
                new Point(scorePose),
                new Point(4, 19),
                new Point(65, 49),
                new Point(sample1)
        ));
        scoreToSample1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading());

        // Path 3: Point B to Park Pose
        sample1ToHuman = new Path(new BezierLine(new Point(sample1), new Point(leaveSample)));
        sample1ToHuman.setConstantHeadingInterpolation(leaveSample.getHeading());

        grabToScoreSpecimen = new Path(new BezierLine(new Point(grabSpecimen), new Point(scorePose)));
        grabToScoreSpecimen.setLinearHeadingInterpolation(grabSpecimen.getHeading(), scorePose.getHeading());

        // Path Chain: Combines Path 3 and a return path
        humanToSample2ToHuman = follower.pathBuilder()
                .addPath(new BezierLine(new Point(leaveSample), new Point(leaveSample)))
                .setConstantHeadingInterpolation(leaveSample.getHeading())
                .addPath(new BezierLine(new Point(leaveSample), new Point(sample2)))
                .setConstantHeadingInterpolation(leaveSample.getHeading())
                .addPath(new BezierLine(new Point(sample2), new Point(leaveSample)))
                .build();
        leaveToWaitToGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(leaveSample), new Point(waitforHuman)))
                .setConstantHeadingInterpolation(waitforHuman.getHeading())
                .addPath(new BezierLine(new Point(waitforHuman), new Point(grabSpecimen)))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(scoreToSample1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(sample1ToHuman);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(humanToSample2ToHuman);
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(leaveToWaitToGrab);
                    setPathState(5);
                }
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(grabToScoreSpecimen);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}

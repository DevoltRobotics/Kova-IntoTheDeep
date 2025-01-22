package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "PedroPathAuto", group = "PedroPath")
@Disabled
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

    private Hardware hardware = new Hardware();

    private PathChain pathChain;

    public void buildPaths() {
        Path scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        Path scoreToSample1 = new Path(new BezierCurve(
                new Point(scorePose),
                new Point(4, 19),
                new Point(65, 49),
                new Point(sample1)
        ));
        scoreToSample1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading());

        // Path 3: Point B to Park Pose
        Path sample1ToHuman = new Path(new BezierLine(new Point(sample1), new Point(leaveSample)));
        sample1ToHuman.setConstantHeadingInterpolation(leaveSample.getHeading());

        Path grabToScoreSpecimen = new Path(new BezierLine(new Point(grabSpecimen), new Point(scorePose)));
        grabToScoreSpecimen.setLinearHeadingInterpolation(grabSpecimen.getHeading(), scorePose.getHeading());

        pathChain = follower.pathBuilder()
                .addTemporalCallback(0, () -> {
                    hardware.liftWristSubsystem.liftWristToPosCmd(1950).schedule();
                })
                .addTemporalCallback(0.8, () -> {
                    hardware.slideSubsystem.liftToPosCmd(-950).schedule();
                })

                .addPath(scorePreload)

                .addTemporalCallback(0, () -> {
                    hardware.wristSubsystem.wristDownCmd().schedule();
                })
                .addTemporalCallback(0.8, () -> {
                    hardware.clawSubsystem.openCmd().schedule();
                })

                .addPath(scoreToSample1)
                .addPath(sample1ToHuman)
                .addPath(grabToScoreSpecimen)

                .addPath(new BezierLine(new Point(leaveSample), new Point(sample1)))
                .setConstantHeadingInterpolation(leaveSample.getHeading())
                .addPath(new BezierLine(new Point(sample1), new Point(sample2)))
                .setConstantHeadingInterpolation(leaveSample.getHeading())
                .addPath(new BezierLine(new Point(sample2), new Point(leaveSample)))
                .addPath(new BezierLine(new Point(leaveSample), new Point(waitforHuman)))
                .setConstantHeadingInterpolation(waitforHuman.getHeading())
                .addPath(new BezierLine(new Point(waitforHuman), new Point(grabSpecimen)))

                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        hardware.init(hardwareMap);

        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        follower.followPath(pathChain);

        hardware.clawSubsystem.closeCmd().schedule();
        hardware.wristSubsystem.wristUpCmd().schedule();
    }

    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void loop() {
        follower.update();

        CommandScheduler.getInstance().run();

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

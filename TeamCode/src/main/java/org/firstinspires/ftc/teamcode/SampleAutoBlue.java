package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.PedroSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(group = "###PedroPath")
public class SampleAutoBlue extends OpMode {

    Hardware hardware = new Hardware();
    PedroSubsystem pedroSubsystem;

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    private Command pathCommand;

    private final Pose startPose = new Pose(9, 87, Math.toRadians(0));

    private final Pose scorePose = new Pose(17, 126, Math.toRadians(135));

    private final Pose scoreSpecimen = new Pose(35,78, Math.toRadians(0));

    private final Pose sample1 = new Pose(35,121, Math.toRadians(0));
    private final Pose sample1Control = new Pose(20,102, Math.toRadians(0));

    private final Pose sample2 = new Pose(35,132, Math.toRadians(0));

    private final Pose sample3 = new Pose(58, 135, Math.toRadians(90));
    private final Pose sample3Control = new Pose(47,117, Math.toRadians(0));

    private final Pose slideSample = new Pose(25,135, Math.toRadians(90));

    private final Pose park = new Pose(59, 97, Math.toRadians(90));
    private final Pose parkControl = new Pose(65, 126, Math.toRadians(90));


    private Path scorePreLoad, sample1Path, scoreSample1, sample2Path, scoreSample2, sample3Path, scoreSample3, slideToPark;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreLoad = new Path(new BezierLine(new Point (startPose), new Point(scoreSpecimen)));
        scorePreLoad.setConstantHeadingInterpolation(startPose.getHeading());

        sample1Path = new Path(new BezierCurve(new Point(scoreSpecimen), new Point(sample1Control), new Point(sample1)));
        sample1Path.setConstantHeadingInterpolation(scoreSpecimen.getHeading());

        scoreSample1 = new Path(new BezierLine(new Point(sample1), new Point(scorePose)));
        scoreSample1.setLinearHeadingInterpolation(sample1.getHeading(), scorePose.getHeading());

        sample2Path = new Path(new BezierLine(new Point(scorePose), new Point(sample2)));
        sample2Path.setLinearHeadingInterpolation(scorePose.getHeading(), sample2.getHeading());

        scoreSample2 = new Path(new BezierLine(new Point(sample2), new Point(scorePose)));
        scoreSample2.setLinearHeadingInterpolation(sample2.getHeading(), scorePose.getHeading());

        sample3Path = new Path(new BezierCurve(new Point(scorePose), new Point(sample3Control), new Point(sample3)));
        sample3Path.setLinearHeadingInterpolation(scorePose.getHeading(), sample3.getHeading());

        scoreSample3 = new Path(new BezierLine(new Point(sample3), new Point(slideSample)));
        scoreSample3.setConstantHeadingInterpolation(sample3.getHeading());

        slideToPark = new Path(new BezierCurve(new Point(slideSample), new Point(parkControl), new Point(park)));
        slideToPark.setConstantHeadingInterpolation(slideSample.getHeading());


        pathCommand = new SequentialCommandGroup(

            new ParallelDeadlineGroup(
                pedroSubsystem.followPathCmd(scorePreLoad)
            ),
            new ParallelDeadlineGroup(
                pedroSubsystem.followPathCmd(sample1Path)
            ),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(scoreSample1)
                ),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(sample2Path)
                ),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(scoreSample2)
                ),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(sample3Path)
                ),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(scoreSample3)
                ),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(slideToPark)
                )
        );
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        // Feedback to Driver Hub
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Follower Path", follower.getCurrentPath());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        hardware.init(hardwareMap, true);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        pedroSubsystem = new PedroSubsystem(follower);
        CommandScheduler.getInstance().registerSubsystem(pedroSubsystem);

        buildPaths();

        hardware.clawSubsystem.closeCmd().schedule();
        hardware.wristSubsystem.wristUpCmd().schedule();
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathCommand.schedule();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

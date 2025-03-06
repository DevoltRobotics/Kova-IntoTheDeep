package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.PedroSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@Autonomous(group = "###PedroPath")
public class SpecimenAutoBlueOptimized extends OpMode {

    Hardware hardware = new Hardware();
    PedroSubsystem pedroSubsystem;

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    private Command pathCommand;

    private final Pose startPose = new Pose(9, 56, Math.toRadians(0));

    private final Pose scorePose = new Pose(35, 69, Math.toRadians(0));

    private final Pose sample1 = new Pose(59, 23, Math.toRadians(180));
    private final Pose scoreTosampleControl1 = new Pose(4,19, Math.toRadians(180));
    private final Pose scoreTosampleControl2 = new Pose(65 ,49, Math.toRadians(180));

    private final Pose sample2 = new Pose(57, 12, Math.toRadians(180));

    private final Pose sample3 = new Pose(58, 10, Math.toRadians(90));

    private final Pose sample3Human = new Pose(16, 10, Math.toRadians(90));

    private final Pose grabSpecimen = new Pose(14.5,22,Math.toRadians(180));

    private final Pose grabSpecimen2 = new Pose(14.5, 22, Math.toRadians(180));
    private final Pose grabSpecimen2Control = new Pose(29, 17, Math.toRadians(180));

    private final Pose scorePose2 = new Pose(37 , 60, Math.toRadians(0));

    private final Pose scorePose3 = new Pose(38.5 , 69, Math.toRadians(0));

    private final Pose scorePose4 = new Pose(38.5 , 66, Math.toRadians(0));

    private final Pose park = new Pose(14.5,22,Math.toRadians(180));


    private final Pose scoreToGrabControl1 = new Pose(35,18,Math.toRadians(180));


    private Path sample2ToSample3, score2ToGrabCurve,sample1ToSample2, leaveSample3,humanToGrab,scorePreload, goToSpecimen1, scoreToGrab, scoreToPark, grabToScore2, grabToScore3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        goToSpecimen1 = new Path(new BezierCurve(new Point (scorePose), new Point(scoreTosampleControl1), new Point(scoreTosampleControl2), new Point(sample1)));
        goToSpecimen1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading());

        sample1ToSample2 = new Path(new BezierLine(new Point(sample1), new Point(sample2)));
        sample1ToSample2.setConstantHeadingInterpolation(sample2.getHeading());

        sample2ToSample3 = new Path(new BezierLine(new Point(sample2), new Point(sample3)));
        sample2ToSample3.setLinearHeadingInterpolation(sample2.getHeading(),sample3.getHeading());

        leaveSample3 = new Path(new BezierLine(new Point(sample3), new Point(sample3Human)));
        leaveSample3.setConstantHeadingInterpolation(sample3.getHeading());

        humanToGrab = new Path(new BezierCurve(new Point(sample3Human), new Point(grabSpecimen2Control), new Point(grabSpecimen2)));
        humanToGrab.setLinearHeadingInterpolation(sample3Human.getHeading(), grabSpecimen2.getHeading());

        score2ToGrabCurve = new Path(new BezierCurve(new Point(scorePose2), new Point(scoreToGrabControl1), new Point(grabSpecimen2)));
        score2ToGrabCurve.setLinearHeadingInterpolation(scorePose2.getHeading(), grabSpecimen.getHeading());

        grabToScore2 = new Path(new BezierLine(new Point(grabSpecimen), new Point(scorePose3)));
        grabToScore2.setLinearHeadingInterpolation(grabSpecimen.getHeading(), scorePose3.getHeading());

        grabToScore3 = new Path(new BezierLine(new Point(grabSpecimen), new Point(scorePose4)));
        grabToScore3.setLinearHeadingInterpolation(grabSpecimen.getHeading(), scorePose4.getHeading());

        scoreToGrab = new Path(new BezierLine(new Point(scorePose), new Point(grabSpecimen)));
        scoreToGrab.setLinearHeadingInterpolation(scorePose.getHeading(), grabSpecimen.getHeading());

        scoreToPark = new Path(new BezierLine(new Point(scorePose), new Point(park)));
        scoreToPark.setLinearHeadingInterpolation(scorePose.getHeading(), grabSpecimen.getHeading());

        pathCommand = new SequentialCommandGroup(
                /* PRIMER SPECIMEN */

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(scorePreload),

                        hardware.liftWristSubsystem.liftWristToPosCmd(1600),
                        hardware.slideSubsystem.slideToPosCmd(-1350),
                        hardware.wristSubsystem.wristUpCmd()
                ),

                new ParallelRaceGroup(
                        new WaitCommand(1100),

                        hardware.wristSubsystem.wristDownCmd(),
                        hardware.liftWristSubsystem.liftWristToPosCmd(1000),
                        hardware.slideSubsystem.slideToPosCmd(-1200)
                ),

                new WaitCommand(1100),
                hardware.clawSubsystem.openCmd(),
                hardware.wristSubsystem.wristUpCmd(),

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(goToSpecimen1),

                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
                        hardware.slideSubsystem.slideToPosCmd(0)
                ),

                //TODO: SISTEMA DE DEJAR PRIMER SAMPLE
                //Aqui ya lo solto

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(sample1ToSample2),

                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
                        hardware.slideSubsystem.slideToPosCmd(0),
                        hardware.wristSubsystem.wristMidCmd()
                ),
                //TODO: SISTEMA DE DEJAR SEGUNDO SAMPLE

                pedroSubsystem.followPathCmd(sample2ToSample3),
                new WaitCommand(450),
                hardware.wristSubsystem.wristMidCmd(),

                pedroSubsystem.followPathCmd(leaveSample3),

                pedroSubsystem.followPathCmd(humanToGrab)

//                new ParallelRaceGroup(
//                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
//                    hardware.slideSubsystem.slideToPosCmd(0)
//                ),
//
//                new ParallelRaceGroup(
//                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
//                        hardware.slideSubsystem.slideToPosCmd(0),
//                        hardware.clawSubsystem.closeCmd()
//                ),
//                new WaitCommand(300),
//                hardware.wristSubsystem.wristUpCmd(),
//                new ParallelDeadlineGroup(
//                        pedroSubsystem.followPathCmd(grabToScore2),
//
//                        hardware.liftWristSubsystem.liftWristToPosCmd(2000),
//                        hardware.slideSubsystem.slideToPosCmd(-1450),
//                        hardware.wristSubsystem.wristUpCmd()
//                ),
//                new ParallelRaceGroup(
//                        new WaitCommand(500),
//
//                        hardware.wristSubsystem.wristDownCmd(),
//                        hardware.liftWristSubsystem.liftWristToPosCmd(300),
//                        hardware.slideSubsystem.slideToPosCmd(-1800)
//                ),
//                new WaitCommand(1100),
//
//                hardware.clawSubsystem.openCmd(),
//                hardware.wristSubsystem.wristUpCmd(),
//
//                new WaitCommand(100),
//                hardware.wristSubsystem.wristMidCmd(),
//
//                new ParallelRaceGroup(
//                        pedroSubsystem.followPathCmd(score2ToGrabCurve),
//
//                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
//                        hardware.slideSubsystem.slideToPosCmd(0)
//                ),
//
//                new ParallelRaceGroup(
//                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
//                        hardware.slideSubsystem.slideToPosCmd(0),
//                        hardware.clawSubsystem.closeCmd()
//                ),
//                new WaitCommand(300),
//                hardware.wristSubsystem.wristUpCmd(),
//                new ParallelDeadlineGroup(
//                        pedroSubsystem.followPathCmd(grabToScore3),
//
//                        hardware.liftWristSubsystem.liftWristToPosCmd(2000),
//                        hardware.slideSubsystem.slideToPosCmd(-1450),
//                        hardware.wristSubsystem.wristUpCmd()
//                ),
//                new ParallelRaceGroup(
//                new WaitCommand(500),
//
//                hardware.wristSubsystem.wristDownCmd(),
//                hardware.liftWristSubsystem.liftWristToPosCmd(300),
//                hardware.slideSubsystem.slideToPosCmd(-1800)
//            ),
//                new WaitCommand(1100),
//
//                hardware.clawSubsystem.openCmd(),
//                hardware.wristSubsystem.wristUpCmd(),
//
//                new WaitCommand(100),
//                hardware.wristSubsystem.wristMidCmd()
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

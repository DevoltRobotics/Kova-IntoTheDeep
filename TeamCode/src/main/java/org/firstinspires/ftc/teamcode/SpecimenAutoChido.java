package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.PedroSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(group = "###PedroPath")
public class SpecimenAutoChido extends OpMode {

    Hardware hardware = new Hardware();
    PedroSubsystem pedroSubsystem;

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    private Command pathCommand;

    private final Pose startPose = new Pose(9, 56, Math.toRadians(0));

    private final Pose scorePose = new Pose(36, 69, Math.toRadians(0));

    private final Pose parkPose = new Pose(9, 33, Math.toRadians(0));

    private final Pose sample1 = new Pose(59, 23, Math.toRadians(180));
    private final Pose scoreTosampleControl1 = new Pose(4,19, Math.toRadians(180));
    private final Pose scoreTosampleControl2 = new Pose(65 ,49, Math.toRadians(180));

    private final Pose sample2 = new Pose(57, 9, Math.toRadians(180));
    private final Pose humanToSample2Control1 = new Pose(67,32, Math.toRadians(180));

    private final Pose scorePose2 = new Pose(40, 69, Math.toRadians(0));


    private final Pose sample3 = new Pose(59, 6, Math.toRadians(90));

    // private final Pose waitForHuman = new Pose(28, 22, Math.toRadians(180));

    private final Pose humanSample = new Pose(17, 25, Math.toRadians(180));

    private final Pose grabSpecimen = new Pose(14.5,22,Math.toRadians(180));

    private final Pose park = new Pose(14.5,22,Math.toRadians(180));

    private final Pose scoreToGrabControl1 = new Pose(35,18,Math.toRadians(180));


    private Path score2ToGrabCurve, humanToSample2,scorePreload, goToSpecimen1, leaveSample1, leaveSample2, grabToScore, scoreToGrab, scoreToPark;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        goToSpecimen1 = new Path(new BezierCurve(new Point (scorePose), new Point(scoreTosampleControl1), new Point(scoreTosampleControl2), new Point(sample1)));
        goToSpecimen1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading());

        leaveSample1 = new Path(new BezierLine(new Point(sample1), new Point(humanSample)));
        leaveSample1.setConstantHeadingInterpolation(humanSample.getHeading());

        humanToSample2 = new Path(new BezierCurve(new Point(humanSample), new Point(humanToSample2Control1), new Point(sample2)));
        humanToSample2.setConstantHeadingInterpolation(humanSample.getHeading());

        leaveSample2 = new Path(new BezierLine(new Point(sample2), new Point(grabSpecimen)));
        leaveSample2.setConstantHeadingInterpolation(humanSample.getHeading());

        grabToScore = new Path(new BezierLine(new Point(grabSpecimen), new Point(scorePose2)));
        grabToScore.setLinearHeadingInterpolation(grabSpecimen.getHeading(), scorePose2.getHeading());

        score2ToGrabCurve = new Path(new BezierCurve(new Point(scorePose2), new Point(scoreToGrabControl1), new Point(grabSpecimen)));
        score2ToGrabCurve.setLinearHeadingInterpolation(scorePose2.getHeading(), grabSpecimen.getHeading());

        scoreToGrab = new Path(new BezierLine(new Point(scorePose), new Point(grabSpecimen)));
        scoreToGrab.setLinearHeadingInterpolation(scorePose.getHeading(), grabSpecimen.getHeading());

        scoreToPark = new Path(new BezierLine(new Point(scorePose), new Point(park)));
        scoreToPark.setLinearHeadingInterpolation(scorePose.getHeading(), grabSpecimen.getHeading());

        pathCommand = new SequentialCommandGroup(
                /* PRIMER SPECIMEN */

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(scorePreload),

                        hardware.liftWristSubsystem.liftWristToPosCmd(1500),
                        hardware.slideSubsystem.liftToPosCmd(-1350),
                        hardware.wristSubsystem.wristUpCmd()
                ),

                new WaitCommand(500),

                new ParallelRaceGroup(
                        new WaitCommand(1100),

                        hardware.wristSubsystem.wristDownCmd(),
                        hardware.liftWristSubsystem.liftWristToPosCmd(1000),
                        hardware.slideSubsystem.liftToPosCmd(-1200)
                ),

                new WaitCommand(1100),
                hardware.clawSubsystem.openCmd(),
                hardware.wristSubsystem.wristUpCmd(),

                /* EMPUJAR SPECIMENS */

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(goToSpecimen1),

                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
                        hardware.slideSubsystem.liftToPosCmd(0)
                ),

                pedroSubsystem.followPathCmd(leaveSample1),

                pedroSubsystem.followPathCmd(humanToSample2),
                hardware.wristSubsystem.wristPosCmd(0.35), //FIXME Checar Coordenadas para agarrar
                pedroSubsystem.followPathCmd(leaveSample2),

                new WaitCommand(200),
                hardware.clawSubsystem.closeCmd(),
                new WaitCommand(450),
                hardware.wristSubsystem.wristUpCmd(),

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(grabToScore),

                        hardware.liftWristSubsystem.liftWristToPosCmd(2000),
                        hardware.slideSubsystem.liftToPosCmd(-1450),
                        hardware.wristSubsystem.wristUpCmd()
                ),

                new ParallelRaceGroup(
                        new WaitCommand(800),

                        hardware.wristSubsystem.wristDownCmd(),
                        hardware.liftWristSubsystem.liftWristToPosCmd(300),
                        hardware.slideSubsystem.liftToPosCmd(-1800)
                ),

                new WaitCommand(1100),
                hardware.clawSubsystem.openCmd(),
                hardware.wristSubsystem.wristUpCmd(),
                hardware.liftWristSubsystem.liftWristToPosCmd(0),
                hardware.slideSubsystem.liftToPosCmd(0),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        pedroSubsystem.followPathCmd(score2ToGrabCurve),
                        hardware.wristSubsystem.wristMidCmd(),
                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
                        hardware.slideSubsystem.liftToPosCmd(-200)
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

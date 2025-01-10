package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    private final Pose startPose = new Pose(9, 56, Math.toRadians(0)); //ToDo Checar orientacion

    private final Pose scorePose = new Pose(35.5, 69, Math.toRadians(0));

    private final Pose parkPose = new Pose(9, 33, Math.toRadians(0));

    private final Pose sample1 = new Pose(59, 23, Math.toRadians(180));
    private final Pose sample2 = new Pose(59, 14, Math.toRadians(180));
    private final Pose sample3 = new Pose(59, 9, Math.toRadians(90));

    private final Pose waitForHuman = new Pose(28, 22, Math.toRadians(180));

    private final Pose humanSample = new Pose(17, 22, Math.toRadians(180));

    private final Pose scoreTosampleControl1 = new Pose(4,19, Math.toRadians(180));
    private final Pose scoreTosampleControl2 = new Pose(65 ,49, Math.toRadians(180));

    private final Pose grabSpecimen = new Pose(10,22,Math.toRadians(180));


    private Path scorePreload, goToSpecimen1, leaveSample2, waitForHumanSpecimen, goToSpecimen2from1,specimen2ToGrab, grabToScore, scoreToGrab;

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

        leaveSample2 = new Path(new BezierLine(new Point(sample2), new Point(humanSample)));
        leaveSample2.setConstantHeadingInterpolation(humanSample.getHeading());

        goToSpecimen2from1 = new Path(new BezierLine(new Point(sample1), new Point (sample2)));
        goToSpecimen2from1.setConstantHeadingInterpolation(sample1.getHeading());


        waitForHumanSpecimen = new Path(new BezierLine(new Point(humanSample), new Point(waitForHuman)));
        waitForHumanSpecimen.setConstantHeadingInterpolation(humanSample.getHeading());

        specimen2ToGrab = new Path(new BezierLine(new Point(sample2), new Point(grabSpecimen)));
        specimen2ToGrab.setConstantHeadingInterpolation(sample2.getHeading());

        grabToScore = new Path(new BezierLine(new Point(grabSpecimen), new Point(scorePose)));
        grabToScore.setLinearHeadingInterpolation(grabSpecimen.getHeading(), scorePose.getHeading());

        scoreToGrab = new Path(new BezierLine(new Point(scorePose), new Point(grabSpecimen)));
        scoreToGrab.setLinearHeadingInterpolation(scorePose.getHeading(), grabSpecimen.getHeading());


        pathCommand = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(scorePreload),

                        hardware.liftWristSubsystem.liftWristToPosCmd(1950),
                        hardware.liftSubsystem.liftToPosCmd(-950)
                ),

                new WaitCommand(500),

                hardware.wristSubsystem.wristDownCmd(),
                new WaitCommand(800),
                hardware.clawSubsystem.openCmd(),
                hardware.wristSubsystem.wristUpCmd(),

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(goToSpecimen1),

                        hardware.liftWristSubsystem.liftWristToPosCmd(0),
                        hardware.liftSubsystem.liftToPosCmd(0)
                ),

                pedroSubsystem.followPathCmd(leaveSample2),
                pedroSubsystem.followPathCmd(goToSpecimen1),
                pedroSubsystem.followPathCmd(goToSpecimen2from1),
                pedroSubsystem.followPathCmd(specimen2ToGrab),
                pedroSubsystem.followPathCmd(grabToScore),
                pedroSubsystem.followPathCmd(scoreToGrab)
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

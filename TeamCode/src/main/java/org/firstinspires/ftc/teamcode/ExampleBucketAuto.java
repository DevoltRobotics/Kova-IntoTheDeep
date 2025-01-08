package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Example Auto Blue", group = "PedroPath")
public class ExampleBucketAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(9, 56, Math.toRadians(0)); //ToDo Checar orientacion

    private final Pose scorePose = new Pose(37, 69, Math.toRadians(0));

    private final Pose parkPose = new Pose(9, 33, Math.toRadians(0));

    private final Pose sample1 = new Pose(59, 23, Math.toRadians(180));
    private final Pose sample2 = new Pose(59, 14, Math.toRadians(180));
    private final Pose sample3 = new Pose(59, 9, Math.toRadians(90));

    private final Pose waitForHuman = new Pose(28, 22, Math.toRadians(180));

    private final Pose humanSample = new Pose(17, 22, Math.toRadians(180));

    private final Pose scoreTosampleControl1 = new Pose(4,19, Math.toRadians(180));
    private final Pose scoreTosampleControl2 = new Pose(65 ,49, Math.toRadians(180));




    private Path scorePreload, goToSpecimen1, leaveSample2,waitForHumanSpecimen;
    private PathChain  leaveSample, goToSpecimen2;

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

        waitForHumanSpecimen = new Path(new BezierLine(new Point(humanSample), new Point(waitForHuman)));
        waitForHumanSpecimen.setConstantHeadingInterpolation(humanSample.getHeading());


        goToSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(sample2)))
                .setConstantHeadingInterpolation(humanSample.getHeading())
                .addPath(new BezierLine(new Point(humanSample), new Point(sample1)))
                .setConstantHeadingInterpolation(humanSample.getHeading())
                .build();

        leaveSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(humanSample)))
                .setConstantHeadingInterpolation(humanSample.getHeading())
                .build();
     }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                if(!follower.isBusy()) {
                    follower.followPath(goToSpecimen1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(leaveSample,false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(goToSpecimen2,true);
                    setPathState(4);
                }
                break;
            case 4:
                    if(!follower.isBusy()) {
                        follower.followPath(leaveSample2);
                        setPathState(5);
                    }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(waitForHumanSpecimen, true);

                    setPathState(6);
                }
                break;
            case 6:



                break;
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(park,true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
//                    /* Level 1 Ascent */
//
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="colgadatest", group="Main Teleop")
public class colgadaTest extends LinearOpMode {

    public PIDFController.PIDCoefficients climbCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.0017);

    PIDFController controllerRight = new PIDFController(climbCoefficients);
    PIDFController controllerLeft = new PIDFController(climbCoefficients);

    public static final double rielesP = 0.004;
    public static final double rielesF = 0.09;

    int rielesTargetPos = 0;
    int centroTargetPos = 0;
    double speed = 1;

    boolean colgada = false;

    ElapsedTime rielesPosTimer = new ElapsedTime();


    ElapsedTime subirServosTimer = new ElapsedTime();
    boolean subirServos = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware hdw = new Hardware();

        hdw.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            boolean manualUp = gamepad1.right_trigger > 0.5;
            boolean manualDown = gamepad1.left_trigger > 0.5;

            telemetry.addData("Manual Up", manualUp);
            telemetry.addData("Manual Dowb", manualDown);
            if (manualUp || manualDown) {
                controllerRight.targetPosition = hdw.rightCM.getCurrentPosition();
                controllerLeft.targetPosition = hdw.leftCM.getCurrentPosition();

                hdw.leftCM.setPower(-gamepad1.right_trigger);
                hdw.rightCM.setPower(gamepad1.right_trigger);
                hdw.leftCM.setPower(gamepad1.left_trigger);
                hdw.rightCM.setPower(-gamepad1.left_trigger);
            } else {
                hdw.leftCM.setPower(controllerLeft.update(hdw.leftCM.getCurrentPosition()));
                hdw.rightCM.setPower(controllerRight.update(hdw.rightCM.getCurrentPosition()));
            }

            if (gamepad1.right_bumper) {
                controllerRight.targetPosition = -5700;
                controllerLeft.targetPosition = 5700;

                subirServos = true;
                subirServosTimer.reset();


            } else if (gamepad1.left_bumper) {
                controllerRight.targetPosition = 0;
                controllerLeft.targetPosition = 0;
            }

            if (subirServos && subirServosTimer.seconds() > 2) {
                hdw.servosUp();
                subirServos = false;
            }

            if (gamepad1.a) {
                hdw.servosDown();

            } else if (gamepad1.y) {
                hdw.servosUp();
            }

        }
    }

}





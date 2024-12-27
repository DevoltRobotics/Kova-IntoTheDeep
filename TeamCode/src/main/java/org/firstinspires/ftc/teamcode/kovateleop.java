package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Kova🎀", group="Linear OpMode")
public class kovateleop extends LinearOpMode {

    public static final double rielesP = 0.004;
    public static final double rielesF = 0.09;

    int rielesTargetPos = 0;
    int centroTargetPos = 0;

    ElapsedTime rielesPosTimer = new ElapsedTime();

    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");
        DcMotor slidesMotor = hardwareMap.dcMotor.get("MR");
        DcMotor centralMotor = hardwareMap.dcMotor.get("C");

        Servo servoGarra = hardwareMap.servo.get("CG");
        Servo servoWrist = hardwareMap.servo.get("CM");

        servoWrist.setDirection(Servo.Direction.REVERSE);

        RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class,"TS");




        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        centralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centralMotor.setTargetPosition(0);
        centralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double turbo = 1;

            if(gamepad1.left_trigger > 0.1) {
                turbo -= (gamepad1.left_trigger * 0.8);
            } else {
                turbo -= (gamepad1.right_trigger * 0.8);
            }

            frontLeftMotor.setPower(frontLeftPower * turbo);
            backLeftMotor.setPower(backLeftPower * turbo);
            frontRightMotor.setPower(frontRightPower * turbo);
            backRightMotor.setPower(backRightPower * turbo);

            rielesTargetPos += (int) (gamepad2.left_stick_y * 50);

            if(touchSensor.isPressed()){
                centralMotor.setPower(0);
            } else {
                centralMotor.setPower(1);
                centroTargetPos += (int) (gamepad2.right_stick_y * 25);
            }

            if(gamepad2.b) {
                servoGarra.setPosition(0.2);
            } else if(gamepad2.a){
                servoGarra.setPosition(1);
            }

            if(gamepad2.dpad_up) { // automatizacion movimientos de muñeca
                servoWrist.setPosition(0);
            } else if(gamepad2.dpad_right) {
                servoWrist.setPosition(0.7);
            } else if(gamepad2.dpad_down) {
                servoWrist.setPosition(1);
            }

            if(gamepad2.x) { // automatizacion bajar rieles y guardar garra
                servoWrist.setPosition(0);
            } else if(gamepad2.y) {
                servoWrist.setPosition(0.7);
            }

            servoWrist.setPosition(servoWrist.getPosition() - ((gamepad2.right_trigger - gamepad2.left_trigger) * 0.05));



            double rielesError = rielesTargetPos - slidesMotor.getCurrentPosition();
            double rielesProportional = rielesError * rielesP;

            slidesMotor.setPower(rielesProportional + rielesF);

            centralMotor.setTargetPosition(centroTargetPos);

            if(rielesPosTimer.milliseconds() > 2000) {
                rielesPosTimer.reset();
                rielesTargetPos = slidesMotor.getCurrentPosition();
                centroTargetPos = centralMotor.getCurrentPosition();
            }

            telemetry.addData("TS",touchSensor.isPressed());
            telemetry.addData("encoderrelies",slidesMotor.getCurrentPosition());
            telemetry.addData("encodercentral",centralMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
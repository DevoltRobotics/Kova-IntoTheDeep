package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Kova🎀 Centric", group="Linear OpMode")
public class KovaTeleOpCentric extends LinearOpMode {

    public static final double rielesP = 0.004;
    public static final double rielesF = 0.09;

    int rielesTargetPos = 0;
    int centroTargetPos = 0;
    double speed = 1;

    ElapsedTime rielesPosTimer = new ElapsedTime();
        @Override
        public void runOpMode() throws InterruptedException {

            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FR");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("BR");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("FL");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("BL");
            DcMotor slidesMotor = hardwareMap.dcMotor.get("MR");
            DcMotor centralMotor = hardwareMap.dcMotor.get("C");
            Servo servoGarra = hardwareMap.servo.get("CG");
            Servo servoWrist = hardwareMap.servo.get("CM");

            slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            centralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class,"TS");

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            centralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            centralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            IMU imu = hardwareMap.get(IMU.class, "imu");

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
            imu.initialize(parameters);

            servoWrist.getController().pwmDisable();

            waitForStart();

            if (isStopRequested()) return;

            slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            centralMotor.setTargetPosition(0);
            centralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            servoWrist.getController().pwmEnable();
            servoWrist.setPosition(0);

            while (opModeIsActive()) {
                if(gamepad1.right_trigger>0.5){
                    speed = 0.25;
                }
                if(gamepad1.right_trigger<0.5){
                    speed = 1;
                }

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

                if (centralMotor.getCurrentPosition() < 260) {
                    rielesTargetPos = Range.clip(rielesTargetPos, -300, 2575);
                    telemetry.addLine("extension limitada");
                }

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

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower * speed);
                backLeftMotor.setPower(backLeftPower * speed);
                frontRightMotor.setPower(frontRightPower * speed);
                backRightMotor.setPower(backRightPower * speed);
            }
        }
    }
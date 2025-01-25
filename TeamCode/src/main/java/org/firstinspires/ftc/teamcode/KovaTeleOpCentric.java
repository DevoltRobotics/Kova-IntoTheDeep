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

@TeleOp(name="Kova🎀 Centric", group="Main Teleop")
public class KovaTeleOpCentric extends LinearOpMode {

    public static final double rielesP = 0.004;
    public static final double rielesF = 0.09;

    int rielesTargetPos = 0;
    int centroTargetPos = 0;
    double speed = 1;

    boolean colgada = false;

    ElapsedTime rielesPosTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hdw = new Hardware();
        hdw.init(hardwareMap);

        IMU imu = hardwareMap.get(IMU.class, "imuc");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        hdw.slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hdw.centralMotor.setTargetPosition(0);
        hdw.centralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hdw.servoWrist.getController().pwmEnable();
        hdw.servoWrist.setPosition(0);

        while (opModeIsActive()) {
            if(!colgada) {
                if (gamepad1.right_trigger > 0.5) {
                    speed = 0.25;
                } else if (gamepad1.right_trigger < 0.5) {
                    speed = 1;
                }
            }
            rielesTargetPos += (int) (gamepad2.left_stick_y * 50);

            if(hdw.touchSensor.isPressed()){
                hdw.centralMotor.setPower(0);
            } else {
                hdw.centralMotor.setPower(1);
                centroTargetPos += (int) (-gamepad2.right_stick_y * 25);
            }

            if(gamepad2.b) {
                hdw.servoGarra.setPosition(0.2);
            } else if(gamepad2.a){
                hdw.servoGarra.setPosition(1);
            }

            if(gamepad2.dpad_up) { // automatizacion movimientos de muñeca
                hdw.servoWrist.setPosition(0);
            } else if(gamepad2.dpad_right) {
                hdw.servoWrist.setPosition(0.35);
            } else if(gamepad2.dpad_down) {
                hdw.servoWrist.setPosition(0.65);
            }

            if(gamepad2.x) { // automatizacion bajar rieles y guardar garra
                hdw.servoWrist.setPosition(0);
            } else if(gamepad2.y) {
                hdw.servoWrist.setPosition(0.7);
            }

            hdw.servoWrist.setPosition(hdw.servoWrist.getPosition() - ((gamepad2.right_trigger - gamepad2.left_trigger) * 0.05));


            if (hdw.centralMotor.getCurrentPosition() < 260) {
                // 333rielesTargetPos = Range.clip(rielesTargetPos, -200, 3900);
                telemetry.addLine("extension limitada");
            }

            double rielesError = rielesTargetPos - hdw.slidesMotor.getCurrentPosition();
            double rielesProportional = rielesError * rielesP;

            hdw.slidesMotor.setPower(rielesProportional + rielesF);

            hdw.centralMotor.setTargetPosition(centroTargetPos);

            if(rielesPosTimer.milliseconds() > 2000) {
                rielesPosTimer.reset();
                rielesTargetPos = hdw.slidesMotor.getCurrentPosition();
                centroTargetPos = hdw.centralMotor.getCurrentPosition();
            }

            if(colgada) {
                hdw.leftCM.setPower(-gamepad1.right_trigger);
                hdw.rightCM.setPower(gamepad1.right_trigger);
                hdw.leftCM.setPower(gamepad1.left_trigger);
                hdw.rightCM.setPower(-gamepad1.left_trigger);

                if (gamepad1.a){
                    hdw.leftC.setPosition(0.5);
                    hdw.rightC.setPosition(0.5);
                }else if(gamepad1.y){
                    hdw.leftC.setPosition(0.6);
                    hdw.rightC.setPosition(0.4);
                }else if (gamepad1.b){
                    hdw.leftC.getController().pwmDisable();
                    hdw.rightC.getController().pwmDisable();
                }else if (gamepad1.x){
                    hdw.leftC.getController().pwmEnable();
                    hdw.rightC.getController().pwmEnable();
                }
            }

            if(gamepad1.dpad_up){
                colgada = !colgada;
            }


            telemetry.addData("Colgada", colgada);

            telemetry.addData("Trigger Value", gamepad1.left_trigger);
            telemetry.addData("Wrist Position", hdw.servoWrist.getPosition());
            telemetry.addData("Motor Power L", hdw.leftCM.getPower());
            telemetry.addData("Motor Power F", hdw.rightCM.getPower());
            telemetry.addData("TS", hdw.touchSensor.isPressed());
            telemetry.addData("encoderrelies", hdw.slidesMotor.getCurrentPosition());
            telemetry.addData("encodercentral", hdw.centralMotor.getCurrentPosition());
            telemetry.update();

            //Centrico Driver 1

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

            hdw.frontLeftMotor.setPower(frontLeftPower * speed);
            hdw.backLeftMotor.setPower(backLeftPower * speed);
            hdw.frontRightMotor.setPower(frontRightPower * speed);
            hdw.backRightMotor.setPower(backRightPower * speed);
        }
    }
}

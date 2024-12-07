/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

@TeleOp(name="Kova", group="Linear OpMode")
public class kovateleop extends LinearOpMode {

    public static final double rielesP = 0.003;
    public static final double rielesF = 0.09;

    int rielesTargetPos = 0;
    int centroTargetPos = 0;

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

            rielesTargetPos += (int) (gamepad2.left_stick_y * 537);

            if(touchSensor.isPressed()){
                centralMotor.setPower(0);
            } else {
                centralMotor.setPower(1);
                centroTargetPos += (int) (gamepad2.right_stick_y * 200);
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
            } else if(gamepad2.dpad_left) {
                centroTargetPos = 100;
            }

            if(gamepad2.x) { // automatizacion bajar rieles y guardar garra
                servoWrist.setPosition(0);
                rielesTargetPos = 0;
            } else if(gamepad2.y) {
                servoWrist.setPosition(0.7);
                rielesTargetPos = 0;
            }

            servoWrist.setPosition(servoWrist.getPosition() - ((gamepad2.right_trigger - gamepad2.left_trigger) * 0.05));

            double rielesError = rielesTargetPos - slidesMotor.getCurrentPosition();
            double rielesProportional = rielesError * rielesP;

            slidesMotor.setPower(rielesProportional + rielesF);

            centralMotor.setTargetPosition(centroTargetPos);

            telemetry.addData("TS",touchSensor.isPressed());
            telemetry.addData("encoderrelies",slidesMotor.getCurrentPosition());
            telemetry.addData("encodercentral",centralMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}
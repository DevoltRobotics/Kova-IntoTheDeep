
package org.firstinspires.ftc.teamcode.autonomokova;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutonomoKovaCanasta extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive MD = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo servoGarra = hardwareMap.servo.get("CG");
        Servo servoWrist = hardwareMap.servo.get("CM");
        DcMotor slidesMotor = hardwareMap.dcMotor.get("MR");
        DcMotor centralMotor = hardwareMap.dcMotor.get("C");

        waitForStart();
        Actions.runBlocking(
                MD.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeTo(new Vector2d(23,13 ))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(2)
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.5)
                        .stopAndAdd(new ServoA(servoWrist, 0.7))
                        .turn(Math.toRadians(-45))
                        .strafeTo(new Vector2d(14, 19))

                        .build());
    }
    public class ServoA implements Action {

        Servo servo;
        double position;
        public ServoA(Servo s, double p){
            this.servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }
}

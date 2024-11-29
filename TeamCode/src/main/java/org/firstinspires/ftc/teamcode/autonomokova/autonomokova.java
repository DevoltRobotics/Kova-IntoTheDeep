package org.firstinspires.ftc.teamcode.autonomokova;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class autonomokova extends LinearOpMode {


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
                        .lineToX(64)
                        .lineToX(0)
                        .build());
    }
}

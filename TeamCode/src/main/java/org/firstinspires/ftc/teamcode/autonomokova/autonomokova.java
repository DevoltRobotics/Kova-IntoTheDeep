package org.firstinspires.ftc.teamcode.autonomokova;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
                        .lineToX(10) // moviemiento para enfrente
                        .strafeTo(new Vector2d(50, -20)) //diagonal
                        .turn(Math.toRadians(-90)) //vuelta
                        .lineToY(-24) //enfrente poquito
                        .strafeTo(new Vector2d(0, -20)) //lleva a human

                        .lineToX(50)
                        .lineToY(-26) //esta parte  no funciona
                        .build());
    }
}


package org.firstinspires.ftc.teamcode.autonomokova;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Autonomo Canasta🔥", group = "Autonomous")
public class AutonomoKovaCanastaPt2 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive MD = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo servoGarra = hardwareMap.servo.get("CG");
        Servo servoWrist = hardwareMap.servo.get("CM");
        DcMotor slidesMotor = hardwareMap.dcMotor.get("MR");
        DcMotor centralMotor = hardwareMap.dcMotor.get("C");

        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO Este autonomo es del la alianza roja, checar alianza azul
        waitForStart();
        Actions.runBlocking(
                MD.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .stopAndAdd(new motorA(slidesMotor, 1))
                        .waitSeconds(0.5)
                        .stopAndAdd(new motorA(slidesMotor,  0))
                        .stopAndAdd(new motorA(centralMotor, 0.5))
                        .strafeTo(new Vector2d(27, -11.5))
                        .waitSeconds(0.4)
                        .stopAndAdd(new motorA(centralMotor, 0))
                        .stopAndAdd(new ServoA(servoWrist,0.9))
                        .waitSeconds(1)
                        .lineToX(17.5)
                        .stopAndAdd(new ServoA(servoGarra, 0)) //AQUI LO DEJO
                        .lineToX(3)
                        .stopAndAdd(new ServoA(servoWrist, 0))
                        .stopAndAdd(new motorA(slidesMotor,-1))
                        .stopAndAdd(new motorA(centralMotor,-0.5))
                        .waitSeconds(0.4)
                        .stopAndAdd(new motorA(slidesMotor,0))
                        .waitSeconds(1)
                        .stopAndAdd(new motorA(centralMotor,0)) //AQUI SE REIBICIO
                        .strafeTo(new Vector2d(33, 30.5))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(1.3)
                        .stopAndAdd(new ServoA(servoGarra, 1)) //AQUI AGARRA EL PRIMER
                        .waitSeconds(0.5)
                        .stopAndAdd(new ServoA(servoWrist, 0.6))
                        .turn(Math.toRadians(-45))
                        .stopAndAdd(new motorA(centralMotor, 1))
                        .stopAndAdd(new motorA(slidesMotor, 1))
                        .waitSeconds(1.8)
                        .stopAndAdd(new motorA(centralMotor, 0))
                        .stopAndAdd(new motorA(slidesMotor, 0))
                        .strafeTo(new Vector2d(12,32))
                        .stopAndAdd(new ServoA(servoGarra, 0))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.5)
                        .stopAndAdd(new motorA(slidesMotor, -1))
                        .stopAndAdd(new motorA(centralMotor, -1))
                        .waitSeconds(1.8)
                        .stopAndAdd(new motorA(slidesMotor, 0))
                        .stopAndAdd(new motorA(centralMotor, 0))
                        .strafeToSplineHeading(new Vector2d(15,40), Math.toRadians(0))
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.5)
                        .stopAndAdd(new ServoA(servoWrist, 0.6))
                        .waitSeconds(30)
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
    public class motorA implements Action {
        private DcMotor motor;
        double power;
        public motorA(DcMotor motor, double power){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            this.motor = motor;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setPower(power);
            return false;
        }
    }
    public class motorEncoder implements Action {
        private DcMotor motor;
        int position;
        public motorEncoder(DcMotor motor, int position){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.motor = motor;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setTargetPosition(position);
            motor.setPower(1);
            return false;
        }
    }

}

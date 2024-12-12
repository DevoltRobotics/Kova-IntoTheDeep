
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
@Autonomous(name = "Autonomo Specimen🔥", group = "Autonomous")
public class AutonomoKovaSpecimen extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        MecanumDrive MD = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo servoGarra = hardwareMap.servo.get("CG");
        Servo servoWrist = hardwareMap.servo.get("CM");
        DcMotor slidesMotor = hardwareMap.dcMotor.get("MR");
        DcMotor centralMotor = hardwareMap.dcMotor.get("C");

        centralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centralMotor.setTargetPosition(0);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        centralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO Este autonomo es del la alianza ROJA, CHECAR alianza AZUL
        // ===Abrir Garra = 0.2, Cerrar Garra = 1===
        // ===Abajo Brazo = 1, Medio = 0.7, Arriba = 0===
        // Cada .waitSeconds es para asegurar que se ejecute la accion

        waitForStart();
        Actions.runBlocking(
                MD.actionBuilder(new Pose2d(0, 0, 0)) //No cambie esto lmao
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .stopAndAdd(new motorEncoder(centralMotor, 1800))
                        .stopAndAdd(new motorEncoder(slidesMotor, 1080))
                        .lineToX(21.5)
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(1.5)
                        .stopAndAdd(new motorEncoder(centralMotor, 700))
                        .stopAndAdd(new ServoA(servoGarra, 0.2))
                        //AQUI YA DEJO EL PRIMER SPECIMEN
                        .stopAndAdd(new motorEncoder(slidesMotor, 0))
                        .stopAndAdd(new motorEncoder(centralMotor, 0))
                        .stopAndAdd(new ServoA(servoWrist, 0.7))
                        .strafeToConstantHeading(new Vector2d(16,-38))//VA AL PRIMERO
                        .strafeToLinearHeading(new Vector2d(28,-39.5), Math.toRadians(0))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.4)
                        .strafeToLinearHeading(new Vector2d(10, -38), Math.toRadians(180))
                        .stopAndAdd(new ServoA(servoGarra, 0.2)) //DEJA SPECIMEN AL HUMAN
                        .stopAndAdd(new ServoA(servoWrist, 0.7))
                        .strafeToLinearHeading(new Vector2d(31, -47.5), Math.toRadians(0))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.4)
                        .strafeToLinearHeading(new Vector2d(10, -38), Math.toRadians(180))
                        .stopAndAdd(new ServoA(servoGarra, 0.2)) //DEJA EL SEGUNDO SPECIMEN
                        .strafeToLinearHeading(new Vector2d(20, -38), Math.toRadians(180)) //RETROCEDE PARA DAR CHANCE AL HUMAN
                        .stopAndAdd(new ServoA(servoWrist, 0.8)) //PREPARA BRAZO
                        .waitSeconds(0.7)
                        .strafeToLinearHeading(new Vector2d(14, -38), Math.toRadians(180))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoGarra, 1)) //AGARRA SPECIMEN
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoWrist, 0))
                        .stopAndAdd(new motorEncoder(centralMotor, 1800))
                        .stopAndAdd(new motorEncoder(slidesMotor, 1080))
                        .strafeToLinearHeading(new Vector2d(27.5, 3), Math.toRadians(0))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.7)
                        .stopAndAdd(new ServoA(servoGarra, 0.2))
                        .stopAndAdd(new motorEncoder(centralMotor, 0))
                        .stopAndAdd(new motorEncoder(slidesMotor, 0))
                        .strafeToLinearHeading(new Vector2d(20, -38), Math.toRadians(180))
                        .stopAndAdd(new ServoA(servoWrist, 0.8))
                        .strafeToLinearHeading(new Vector2d(14, -38), Math.toRadians(180))
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoWrist, 0))
                        .stopAndAdd(new motorEncoder(centralMotor, 1800))
                        .stopAndAdd(new motorEncoder(slidesMotor, 1080))
                        .strafeToLinearHeading(new Vector2d(26, 5), Math.toRadians(0))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.7)
                        .stopAndAdd(new ServoA(servoGarra, 0.2))
                        .waitSeconds(30)
                        .stopAndAdd(new motorEncoder(centralMotor, 0))
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
    public class motorEncoder implements Action {
        private DcMotor motor;
        int position;
        public motorEncoder(DcMotor motor, int position){
            this.motor = motor;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setTargetPosition(position);
            motor.setPower(0.6);
            return false;
        }
    }

}

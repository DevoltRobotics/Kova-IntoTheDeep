
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Auto-Specimen-ROJO 🟥", group = "Autonomous")
public class AutoKovaSpecimenROJO extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        /**---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
        //              ======Este autonomo es del la alianza ROJA======
        //                 ======TODO Cambiar estas variables======
        //           X=Delante y detras /// Y=Lado Negativo Derecha, Positivo Izquierda

        //    Los vectores estan colocados en cada trayectoria para solo modificar directamente aqui, lo que mas falla es la distancia hacia adelante (X)

        double primerAcercamiento = 21.5;
        Vector2d segundoAcercamiento = new Vector2d(28,-39.5);  //TODO Segundo acercamiento para recoger el primer sample en el suelo
        Vector2d tercerAcercamiento = new Vector2d(31, -47.5); //TODO Tercer acercamiento para recoger el segundo sample del suelo
        Vector2d colocarPrimerSpecimen = new Vector2d(27.5, 3); //TODO Distancia para colocar primer specimen
        Vector2d colocarSegundoSpecimen = new Vector2d(26, 5); //TODO Distancia para colocar segundo specimen

        Vector2d dejarSampleAlHuman = new Vector2d(10, -38);
        /**-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

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


        // ===Abrir Garra = 0.2, Cerrar Garra = 1===
        // ===Abajo Brazo = 1, Medio = 0.7, Arriba = 0===
        // Cada .waitSeconds es para asegurar que se ejecute la accion

        waitForStart();
        Actions.runBlocking(
                MD.actionBuilder(new Pose2d(0, 0, 0)) //No cambie esto lmao
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .stopAndAdd(new motorEncoder(centralMotor, 1800))
                        .stopAndAdd(new motorEncoder(slidesMotor, 1080))
                        .lineToX(primerAcercamiento) //Primer acercamiento al robot para dejar el primer Specimen
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(1.5)
                        .stopAndAdd(new motorEncoder(centralMotor, 700))
                        .stopAndAdd(new ServoA(servoGarra, 0.2))
                        //AQUI YA DEJO EL PRIMER SPECIMEN
                        .stopAndAdd(new motorEncoder(slidesMotor, 0))
                        .stopAndAdd(new motorEncoder(centralMotor, 0))
                        .stopAndAdd(new ServoA(servoWrist, 0.7))
                        .strafeToConstantHeading(new Vector2d(16,-38))
                        .strafeToLinearHeading(segundoAcercamiento, Math.toRadians(0)) //Segundo acercamiento para recoger el primer sample en el suelo
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.4)
                        .strafeToLinearHeading(dejarSampleAlHuman, Math.toRadians(180)) //Este valor no fallo nunca pero igual checar, es la rotacion para dejarle el sample al Human
                        .stopAndAdd(new ServoA(servoGarra, 0.2)) //DEJA SAMPLE AL HUMAN
                        .stopAndAdd(new ServoA(servoWrist, 0.7))
                        .strafeToLinearHeading(tercerAcercamiento, Math.toRadians(0)) //Tercer acercamiento para recoger el segundo sample del suelo
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.4)
                        .strafeToLinearHeading(dejarSampleAlHuman, Math.toRadians(180)) //Este valor no fallo nunca pero igual checar, es la rotacion para dejarle el sample al Human
                        .stopAndAdd(new ServoA(servoGarra, 0.2)) //DEJA EL SEGUNDO SAMPLE
                        .strafeToLinearHeading(new Vector2d(20, -38), Math.toRadians(180)) //RETROCEDE PARA DAR CHANCE AL HUMAN
                        .stopAndAdd(new ServoA(servoWrist, 0.72)) //PREPARA BRAZO
                        .waitSeconds(0.7)
                        .strafeToLinearHeading(new Vector2d(14, -38), Math.toRadians(180)) //Distancia para agarrar sample, -NO LE COLOQUE VARIABLE DE VECTOR-
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoGarra, 1)) //AGARRA SPECIMEN
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoWrist, 0))
                        .stopAndAdd(new motorEncoder(centralMotor, 1800))
                        .stopAndAdd(new motorEncoder(slidesMotor, 1080))
                        .strafeToLinearHeading(colocarPrimerSpecimen, Math.toRadians(0)) //Distancia para colocar primer specimen
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.7)
                        .stopAndAdd(new ServoA(servoGarra, 0.2))
                        .stopAndAdd(new motorEncoder(centralMotor, 0))
                        .stopAndAdd(new motorEncoder(slidesMotor, 0))
                        .strafeToLinearHeading(new Vector2d(24, -38), Math.toRadians(180))
                        .stopAndAdd(new ServoA(servoWrist, 0.75))
                        .strafeToLinearHeading(new Vector2d(14, -38), Math.toRadians(180)) //Distancia para agarrar sample, -NO LE COLOQUE VARIABLE DE VECTOR-
                        .stopAndAdd(new ServoA(servoGarra, 1))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoWrist, 0))
                        .stopAndAdd(new motorEncoder(centralMotor, 1800))
                        .stopAndAdd(new motorEncoder(slidesMotor, 1080))
                        .strafeToLinearHeading(colocarSegundoSpecimen, Math.toRadians(0))
                        .stopAndAdd(new ServoA(servoWrist, 1))
                        .waitSeconds(0.4)
                        .stopAndAdd(new ServoA(servoGarra, 0.2))
                        .stopAndAdd(new ServoA(servoWrist,0.7))
                        .stopAndAdd(new motorEncoder(centralMotor, 0))
                        .stopAndAdd(new motorEncoder(slidesMotor, 0))
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

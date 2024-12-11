package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

@TeleOp(name = "Monolito", group = "Linear OpMode")
public class KovaTeleopMonolito extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("FR");
        backLeftMotor = hardwareMap.dcMotor.get("BR");
        frontRightMotor = hardwareMap.dcMotor.get("FL");
        backRightMotor = hardwareMap.dcMotor.get("BL");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        new Thread(() -> {
            while (!isStopRequested()) {
                try (ServerSocket serverSocket = new ServerSocket(9000)) {
                    telemetry.addData("Status", "Waiting for connection...");
                    telemetry.update();

                    Socket clientSocket = serverSocket.accept();
                    telemetry.addData("Status", "Client connected");
                    telemetry.update();

                    BufferedReader input = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                    String command;

                    while ((command = input.readLine()) != null) {
                        telemetry.addData("Received Command", command);
                        telemetry.update();
                        handleCommand(command);
                    }

                    clientSocket.close();
                } catch (Exception e) {
                    telemetry.addData("Error", e.getMessage());
                    telemetry.update();
                }
            }
        }).start();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }
    }

    private void handleCommand(String command) {
        switch (command) {
            case "forward":
                setMotorPowers(1, 1, 1, 1);
                break;
            case "backward":
                setMotorPowers(-1, -1, -1, -1);
                break;
            case "left":
                setMotorPowers(-1, 1, 1, -1);
                break;
            case "right":
                setMotorPowers(1, -1, -1, 1);
                break;
            case "stop":
                setMotorPowers(0, 0, 0, 0);
                break;
            default:
                telemetry.addData("Unknown Command", command);
                telemetry.update();
        }
    }

    // Helper function to set motor powers
    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }
}

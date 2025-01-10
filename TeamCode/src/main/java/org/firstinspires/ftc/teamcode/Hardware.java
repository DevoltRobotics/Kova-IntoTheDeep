package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftWristSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class Hardware {

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor slidesMotor;
    public DcMotor centralMotor;

    public Servo servoGarra;
    public Servo servoWrist;

    public Servo leftC;
    public Servo rightC;
    public DcMotor leftCM;
    public DcMotor rightCM;

    public RevTouchSensor touchSensor;

    public ClawSubsystem clawSubsystem;
    public WristSubsystem wristSubsystem;
    public LiftSubsystem liftSubsystem;
    public LiftWristSubsystem liftWristSubsystem;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, false);
    }

    public void init(HardwareMap hardwareMap, boolean pedro) {
        CommandScheduler.getInstance().reset();

        if(!pedro) {
            frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
            backLeftMotor = hardwareMap.dcMotor.get("leftRear");
            frontRightMotor = hardwareMap.dcMotor.get("rightFront");
            backRightMotor = hardwareMap.dcMotor.get("rightRear");
        }

        slidesMotor = hardwareMap.dcMotor.get("MR");
        centralMotor = hardwareMap.dcMotor.get("C");
        servoGarra = hardwareMap.servo.get("CG");
        servoWrist = hardwareMap.servo.get("CM");

        leftC = hardwareMap.servo.get("scl");
        rightC = hardwareMap.servo.get("scr");
        leftCM = hardwareMap.dcMotor.get("cl");
        rightCM = hardwareMap.dcMotor.get("cr");

        touchSensor = hardwareMap.get(RevTouchSensor.class,"TS");

        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(!pedro) {
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        rightC.setDirection(Servo.Direction.REVERSE);
        leftC.setDirection(Servo.Direction.REVERSE);

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        centralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawSubsystem = new ClawSubsystem(servoGarra);
        wristSubsystem = new WristSubsystem(servoWrist);
        liftSubsystem = new LiftSubsystem(slidesMotor);
        liftWristSubsystem = new LiftWristSubsystem(centralMotor);

        CommandScheduler.getInstance().registerSubsystem(clawSubsystem);
        CommandScheduler.getInstance().registerSubsystem(wristSubsystem);
        CommandScheduler.getInstance().registerSubsystem(liftSubsystem);
        CommandScheduler.getInstance().registerSubsystem(liftWristSubsystem);
    }

}

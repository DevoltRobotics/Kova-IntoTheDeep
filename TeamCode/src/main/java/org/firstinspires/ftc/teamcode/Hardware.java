package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
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
    public Servo light;

    public Servo leftC;
    public Servo rightC;
    public DcMotor leftCM;
    public DcMotor rightCM;

    public RevTouchSensor touchSensor;

    public ClawSubsystem clawSubsystem;
    public WristSubsystem wristSubsystem;
    public SlideSubsystem slideSubsystem;
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

        light = hardwareMap.servo.get("light");

        leftCM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightCM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightCM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftCM.setDirection(DcMotorSimple.Direction.REVERSE);

        touchSensor = hardwareMap.get(RevTouchSensor.class,"TS");

        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoWrist.setDirection(Servo.Direction.REVERSE);

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
        slideSubsystem = new SlideSubsystem(slidesMotor);
        liftWristSubsystem = new LiftWristSubsystem(centralMotor);

        CommandScheduler.getInstance().registerSubsystem(clawSubsystem);
        CommandScheduler.getInstance().registerSubsystem(wristSubsystem);
        CommandScheduler.getInstance().registerSubsystem(slideSubsystem);
        CommandScheduler.getInstance().registerSubsystem(liftWristSubsystem);
    }

    public void servosUp() {
        leftC.setPosition(0.5 + SERVOSUP);
        rightC.setPosition(0.5 - SERVOSUP);
    }

    public void servosDown() {
        ElapsedTime timer = new ElapsedTime();
        leftC.setPosition(0.5 + SERVOSDOWN);
        rightC.setPosition(0.5 - SERVOSDOWN);

        timer.reset();
        if (timer.seconds() > 0.2) {
            leftC.getController().pwmDisable();
        }
    }
}

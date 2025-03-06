package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.Constants.WRISTDOWN;
import static org.firstinspires.ftc.teamcode.Constants.WRISTMID;
import static org.firstinspires.ftc.teamcode.Constants.WRISTUP;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {
    Servo wrist;

    public WristSubsystem(Servo wrist) {
        this.wrist = wrist;
    }

    public Command wristUpCmd() {
        return new WristCmd(WRISTUP);
    }

    public Command wristMidCmd() {
        return new WristCmd(WRISTMID);
    }

    public Command wristDownCmd() {
        return new WristCmd(WRISTDOWN);
    }

    public Command wristPosCmd(double position) {
        return new WristCmd(position);
    }

    class WristCmd extends CommandBase {
        double position;

        public WristCmd(double position) {
            this.position = position;
            addRequirements(WristSubsystem.this);
        }

        @Override
        public void execute() {
            wrist.setPosition(position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}

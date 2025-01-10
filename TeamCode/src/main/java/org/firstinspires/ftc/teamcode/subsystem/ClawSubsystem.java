package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    Servo claw;

    public ClawSubsystem(Servo claw) {
        this.claw = claw;
    }

    public Command openCmd() {
        return new ClawCmd(1);
    }

    public Command closeCmd() {
        return new ClawCmd(0.2);
    }

    class ClawCmd extends CommandBase {
        double position;

        public ClawCmd(double position) {
            this.position = position;
            addRequirements(ClawSubsystem.this);
        }

        @Override
        public void execute() {
            claw.setPosition(position);
        }
    }


}

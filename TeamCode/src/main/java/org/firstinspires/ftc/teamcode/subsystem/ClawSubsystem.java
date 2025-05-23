package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.Constants.CLAWCLOSE;
import static org.firstinspires.ftc.teamcode.Constants.CLAWOPEN;
import static org.firstinspires.ftc.teamcode.Constants.LIGHTGREEN;
import static org.firstinspires.ftc.teamcode.Constants.LIGHTRED;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    Servo claw;
    Servo light;

    public ClawSubsystem(Servo claw) {
        this.claw = claw;
    }

    public Command openCmd() {
        return new ClawCmd(CLAWCLOSE);
    }

    public Command closeCmd() {
        return new ClawCmd(CLAWOPEN);
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

        @Override
        public boolean isFinished() {
            return true;
        }
    }


}

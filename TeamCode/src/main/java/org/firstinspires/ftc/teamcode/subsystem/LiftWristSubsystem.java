package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftWristSubsystem extends SubsystemBase {

    DcMotor liftWristMotor;

    public LiftWristSubsystem(DcMotor liftWristMotor) {
        this.liftWristMotor = liftWristMotor;
    }

    public Command liftWristToPosCmd(double position) {
        return new LiftWristToPosCmd(position);
    }

    class LiftWristToPosCmd extends CommandBase {
        double position;

        public LiftWristToPosCmd(double position) {
            this.position = position;
            addRequirements(LiftWristSubsystem.this);
        }

        @Override
        public void execute() {
            liftWristMotor.setTargetPosition(0);
            liftWristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        @Override
        public void end(boolean interrupted) {
            liftWristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftWristMotor.setPower(0);
        }
    }
}

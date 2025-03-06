package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class SlideSubsystem extends SubsystemBase {

    public static final double rielesP = 0.004;
    public static final double rielesF = 0.09;

    DcMotor liftMotor;

    public SlideSubsystem(DcMotor liftMotor) {
        this.liftMotor = liftMotor;
    }

    public Command slideToPosCmd(double position) {
        return new LiftToPosCmd(position);
    }

    class LiftToPosCmd extends CommandBase {
        double position;

        public LiftToPosCmd(double position) {
            this.position = position;
            addRequirements(SlideSubsystem.this);
        }

        @Override
        public void execute() {
            double rielesError = position - liftMotor.getCurrentPosition();
            double rielesProportional = rielesError * rielesP;

            liftMotor.setPower(rielesProportional + rielesF);
        }

        @Override
        public void end(boolean interrupted) {
            liftMotor.setPower(0);
        }
    }
}

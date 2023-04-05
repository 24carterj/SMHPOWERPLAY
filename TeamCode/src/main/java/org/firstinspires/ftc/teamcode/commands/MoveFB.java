package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class MoveFB extends CommandBase {

    private DriveSubsystem drive;
    private double dist, speed;

    public MoveFB(DriveSubsystem ss, double d, double s) {
        drive = ss;
        dist = d;
        speed = s;
        drive.resetEncoders();
        addRequirements(ss);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
        drive.driveRC(0,Math.signum(dist) * speed,0);
    }

    @Override
    public void end(boolean interrupted) {

        drive.driveRC(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getLeftEncoderRevs() * 4.0 * Math.PI * 6) >= Math.abs(dist);
    }
}

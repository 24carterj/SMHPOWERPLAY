package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


public class MoveLR2 extends CommandBase {

    private DriveSubsystem drive;
    private double dist, speed;

    public MoveLR2(DriveSubsystem ss, double d, double s) {
        drive = ss;
        dist = d;
        speed = s;

        addRequirements(ss);
    }

    @Override
    public void initialize() { drive.resetEncoders(); }

    @Override
    public void execute() {
        drive.driveRC(Math.signum(dist) * speed,0,0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRC(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getRevolutions() * 4.0 * Math.PI) >= Math.abs(dist);
    }
}

/*
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class MoveLR extends CommandBase {

    private DriveSubsystem drive;
    private double dist, speed;

    public MoveLR(DriveSubsystem ss, double d, double s) {
        drive = ss;
        dist = d;
        speed = s;

        addRequirements(ss);
    }

    @Override
    public void initialize() { drive.resetEncoders(); }

    @Override
    public void execute() {
        drive.driveRC(Math.signum(dist) * speed,0,0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRC(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getRevolutions() * 4.0 * Math.PI) >= Math.abs(dist);
    }
}

 */
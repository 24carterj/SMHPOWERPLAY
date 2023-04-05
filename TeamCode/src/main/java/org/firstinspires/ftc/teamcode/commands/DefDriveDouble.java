package org.firstinspires.ftc.teamcode.commands;

// import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefDriveDouble extends CommandBase {

    private final DriveSubsystem driveSS;
    private final double magnitude;
    private final double angle;
    private final double turn;

    public DefDriveDouble(DriveSubsystem subsys, double m, double a, double t) {
        driveSS = subsys;
        magnitude = m;
        angle = a;
        turn = t;

        addRequirements(driveSS);
    }

    @Override
    public void execute() {
        driveSS.driveRC(magnitude, angle, turn);
    }
}

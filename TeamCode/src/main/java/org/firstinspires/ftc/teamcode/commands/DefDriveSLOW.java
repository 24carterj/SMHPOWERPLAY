package org.firstinspires.ftc.teamcode.commands;

// import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

@Disabled
public class DefDriveSLOW extends CommandBase {

    private final DriveSubsystem driveSS;
    private final DoubleSupplier magnitude;
    private final DoubleSupplier angle;
    private final DoubleSupplier turn;
    private DoubleSupplier heading;

    public DefDriveSLOW(DriveSubsystem subsys, DoubleSupplier m, DoubleSupplier a, DoubleSupplier t) {
        driveSS = subsys;
        magnitude = m;
        angle = a;
        turn = t;

        addRequirements(driveSS);
    }

    public DefDriveSLOW(DriveSubsystem subsys, DoubleSupplier m, DoubleSupplier a, DoubleSupplier t, DoubleSupplier h) {
        driveSS = subsys;
        magnitude = m;
        angle = a;
        turn = t;
        heading = h;

        addRequirements(driveSS);
    }


    @Override
    public void execute() {
        driveSS.driveRC(0.6*magnitude.getAsDouble(), 0.6*angle.getAsDouble(), 0.8*turn.getAsDouble()/*, heading.getAsDouble()*/);
    }
}

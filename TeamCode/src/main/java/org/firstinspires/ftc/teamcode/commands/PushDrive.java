package org.firstinspires.ftc.teamcode.commands;

// import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

@Disabled
public class PushDrive extends CommandBase {

    private final DriveSubsystem driveSS;

    private final DoubleSupplier magnitude;
    private final DoubleSupplier angle;
    private final DoubleSupplier turn;
    private       DoubleSupplier heading;

    public PushDrive(DriveSubsystem subsys, DoubleSupplier m, DoubleSupplier a, DoubleSupplier t) {
        driveSS = subsys;
        magnitude = m;
        angle = a;
        turn = t;

        addRequirements(driveSS);
    }

    public PushDrive(DriveSubsystem subsys, DoubleSupplier m, DoubleSupplier a, DoubleSupplier t, DoubleSupplier h) {
        driveSS = subsys;
        magnitude = m;
        angle = a;
        turn = t;
        heading = h;

        addRequirements(driveSS);
    }


    @Override
    public void execute() {
        driveSS.driveRC(-1 * magnitude.getAsDouble(), -1 * angle.getAsDouble(), -1 * turn.getAsDouble()/*, heading.getAsDouble()*/);
    }
}

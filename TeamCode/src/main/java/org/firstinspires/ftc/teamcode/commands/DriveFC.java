package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;
@Disabled

public class DriveFC extends CommandBase {
    private DriveSubsystem drive;
    private double head;
    private GamepadEx gamepad;
    private double x, y, rot, h;

    public DriveFC(DriveSubsystem subsys, DoubleSupplier _x, DoubleSupplier _y, DoubleSupplier _rot, DoubleSupplier _h) {
        drive = subsys;
        x = _x.getAsDouble();
        y = _y.getAsDouble();
        rot = _rot.getAsDouble();
        head = _h.getAsDouble();

        addRequirements(drive);
    }


    @Override
    public void execute() {
        // drive.driveFC(x, y, rot, h);
    }
}

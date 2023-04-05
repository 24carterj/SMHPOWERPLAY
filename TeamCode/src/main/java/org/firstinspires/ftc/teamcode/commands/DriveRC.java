package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveRC extends CommandBase {
    private DriveSubsystem drive;
    // private DoubleSupplier mag, ang, turn, head;
    private GamepadEx gamepad;

    public DriveRC(DriveSubsystem subsys, GamepadEx gp) {
        drive = subsys;
        gamepad = gp;

        addRequirements(drive);
    }


    @Override
    public void execute() {
        drive.driveRC(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX());
    }
}

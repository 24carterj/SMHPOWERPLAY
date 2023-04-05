package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import java.util.concurrent.TimeUnit;

public class ClawGrab extends CommandBase {
    private final ClawSubsystem claw;

    public ClawGrab(ClawSubsystem c) {
        claw = c;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.grab(); // grab cone
    }

    @Override
    public boolean isFinished() {
        // need a way to time movement
        return true;
    }
}

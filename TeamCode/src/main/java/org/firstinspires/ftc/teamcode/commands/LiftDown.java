package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftDown extends CommandBase {

    private final LiftSubsystem lift;

    public LiftDown(LiftSubsystem l) {
        lift = l;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.down();
    }
    /*
    @Override
    public boolean isFinished() {
        return lift.stopConditions();
    }
     */
}

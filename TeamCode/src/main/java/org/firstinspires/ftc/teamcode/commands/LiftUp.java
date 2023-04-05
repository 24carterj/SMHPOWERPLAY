package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftUp extends CommandBase {
    private final LiftSubsystem lift;

    public LiftUp(LiftSubsystem l) {
        lift = l;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.up();
    }
    /*
    @Override
    public boolean isFinished() {
        return lift.stopConditions();
    }
    */
}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TwoLiftSubsystem;

public class LiftUpDist extends CommandBase {

    private final TwoLiftSubsystem lifts;
    private double targetDist;
    private double initDist;

    public LiftUpDist(TwoLiftSubsystem l, double d) {
        lifts = l;
        initDist = lifts.getPos();
        targetDist = d;
        addRequirements(lifts);
    }

    @Override
    public void initialize() {
        lifts.up();
    }

    @Override
    public void end(boolean interrupted) {
        lifts.stop();
    }

    @Override
    public boolean isFinished() {
        return lifts.getRevs()*4.7 >= targetDist;
    }
    /*
    @Override
    public boolean isFinished() {
        return lift.stopConditions();
    }
     */
}

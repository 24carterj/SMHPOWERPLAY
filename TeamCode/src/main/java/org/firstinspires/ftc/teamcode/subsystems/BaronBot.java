package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class BaronBot {

    public enum OpModeType { AUTO, TELE }

    public BaronBot(OpModeType type) {
        if (type == OpModeType.AUTO) initAuto();
        else initTele();
    }

    public void initAuto() {

    }

    public void initTele() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx tool = new GamepadEx(gamepad2);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);

        LiftSubsystem lift = new LiftSubsystem(hardwareMap, "lift");
        ClawSubsystem claw = new ClawSubsystem(hardwareMap, "claw");
    }
}

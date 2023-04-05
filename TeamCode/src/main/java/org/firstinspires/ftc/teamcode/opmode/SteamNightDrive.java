package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Disabled
@TeleOp(name="steam")
public class SteamNightDrive extends CommandOpMode {

    private GamepadEx driver;
    private MecanumDrive mecanumDrive;
    private DriveSubsystem drive;

    private boolean autoDetect;

    @Override
    public void initialize() {

        // Gamepad(s)
        driver = new GamepadEx(gamepad1);

        // Drive
        drive = new DriveSubsystem(hardwareMap);

        // Subsystems
        // claw commands
        drive.setDefaultCommand(new DefDrive(drive, driver::getLeftX, driver::getLeftY, driver::getRightX));


    }
}

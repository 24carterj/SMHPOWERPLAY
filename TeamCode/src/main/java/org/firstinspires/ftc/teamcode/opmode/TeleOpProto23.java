package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.DefDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemFlat;



@Disabled
@TeleOp(name="TeleOpTetst")
public class TeleOpProto23 extends CommandOpMode {
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

        drive.setDefaultCommand(new DefDrive(drive, driver::getLeftX, driver::getLeftY, driver::getRightX));
    }
}

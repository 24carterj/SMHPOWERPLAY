package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawGrab;
import org.firstinspires.ftc.teamcode.commands.ClawRelease;
import org.firstinspires.ftc.teamcode.commands.LiftDown;
import org.firstinspires.ftc.teamcode.commands.LiftUp;
import org.firstinspires.ftc.teamcode.commands.PushDrive;
import org.firstinspires.ftc.teamcode.commands.TwoLiftDown;
import org.firstinspires.ftc.teamcode.commands.TwoLiftUp;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoLiftSubsystem;

@Disabled
@TeleOp(name="Push")
public class MecPush extends CommandOpMode {

    // gamepad
    private GamepadEx driver;
    // drive
    private MecanumDrive mecanumDrive;
    private DriveSubsystem drive;
    // control elements
    private ClawSubsystem clawSS;
    private TwoLiftSubsystem liftSS;

    @Override
    public void initialize() {

        // Gamepad(s)
        driver = new GamepadEx(gamepad1);

        // Drive
        drive = new DriveSubsystem(hardwareMap);

        // Subsystems
        clawSS = new ClawSubsystem(hardwareMap, "claw");
        liftSS = new TwoLiftSubsystem(hardwareMap, 1);

        // claw commands
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ClawGrab(clawSS));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ClawRelease(clawSS));

        // lift commands
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new TwoLiftUp(liftSS))
                .whenReleased(new InstantCommand(liftSS::stop, liftSS));
        // .whenReleased(new InstantCommand(liftSS::hold, liftSS));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new TwoLiftDown(liftSS))
                .whenReleased(new InstantCommand(liftSS::stop, liftSS));
        // .whenReleased(new InstantCommand(liftSS::hold, liftSS));

        // Drive command
        drive.setDefaultCommand(new PushDrive(drive, driver::getLeftX, driver::getLeftY, driver::getRightX));
    }
}

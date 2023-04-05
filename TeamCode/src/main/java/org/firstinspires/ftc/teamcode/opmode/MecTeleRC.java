package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawGrab;
import org.firstinspires.ftc.teamcode.commands.ClawRelease;
import org.firstinspires.ftc.teamcode.commands.DefDrive;
import org.firstinspires.ftc.teamcode.commands.LiftDown;
import org.firstinspires.ftc.teamcode.commands.LiftUp;
import org.firstinspires.ftc.teamcode.commands.TwoLiftDown;
import org.firstinspires.ftc.teamcode.commands.TwoLiftUp;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoLiftSubsystem;

@TeleOp(name="TeleOp_RC")
public class MecTeleRC extends CommandOpMode {

    private GamepadEx driver;
    private MecanumDrive mecanumDrive;

    private DriveSubsystem drive;
    private ClawSubsystem clawSS;
    private TwoLiftSubsystem liftSS;

    private boolean autoDetect;

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
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawGrab(clawSS));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawRelease(clawSS));

        // lift commands
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new TwoLiftUp(liftSS))
                .whenReleased(new InstantCommand(liftSS::stop, liftSS));
                // .whenReleased(new InstantCommand(liftSS::hold, liftSS));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new TwoLiftDown(liftSS))
                .whenReleased(new InstantCommand(liftSS::stop, liftSS));
                // .whenReleased(new InstantCommand(liftSS::hold, liftSS));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(drive::incrSpeed));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(drive::decrSpeed));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(liftSS::incrSpeed));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(liftSS::decrSpeed));

        drive.setDefaultCommand(new DefDrive(drive, driver::getLeftX, driver::getLeftY, driver::getRightX));


    }
}

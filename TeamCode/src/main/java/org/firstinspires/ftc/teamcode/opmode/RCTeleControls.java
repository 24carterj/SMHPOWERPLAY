package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawGrab;
import org.firstinspires.ftc.teamcode.commands.ClawRelease;
import org.firstinspires.ftc.teamcode.commands.DefDrive;
import org.firstinspires.ftc.teamcode.commands.LiftDown;
import org.firstinspires.ftc.teamcode.commands.LiftUp;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Disabled
@TeleOp(name="TeleOp_RC_controls")
public class RCTeleControls extends CommandOpMode {

    private GamepadEx driver;
    private MecanumDrive mecanumDrive;

    private ColorSubsystem colorSS;

    private DriveSubsystem drive;
    private ClawSubsystem clawSS;
    private LiftSubsystem liftSS;

    private boolean autoDetect;

    @Override
    public void initialize() {

        // Gamepad(s)
        driver = new GamepadEx(gamepad1);

        // Drive
        drive = new DriveSubsystem(hardwareMap);

        // Subsystems
        clawSS = new ClawSubsystem(hardwareMap, "claw");
        liftSS = new LiftSubsystem(hardwareMap, "lift");

        colorSS = new ColorSubsystem(hardwareMap, "color");

        // claw commands
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawGrab(clawSS));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawRelease(clawSS));

        // lift commands
        TriggerReader ltrig = new TriggerReader(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader rtrig = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (rtrig.isDown()) schedule(new LiftUp(liftSS));

        if (ltrig.isDown()) schedule(new LiftDown(liftSS));
        if (rtrig.wasJustReleased() || ltrig.wasJustReleased()) schedule(new InstantCommand(liftSS::stop, liftSS));


        drive.setDefaultCommand(new DefDrive(drive, driver::getLeftX, driver::getLeftY, driver::getRightX));


    }
}

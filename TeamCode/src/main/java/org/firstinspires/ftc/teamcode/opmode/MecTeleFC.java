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

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Disabled
@TeleOp(name="TeleFieldCentric")
public class MecTeleFC extends CommandOpMode {
    private GamepadEx driver;
    private MecanumDrive mecanumDrive;

    private Button clawOpenButton;
    private Button clawCloseButton;

    private Trigger liftUpTrigger;
    private Trigger liftDownTrigger;

    private ClawSubsystem clawSS;
    private LiftSubsystem liftSS;

    @Override
    public void initialize() {
        // Gamepad(s)
        driver = new GamepadEx(gamepad1);

        // Subsystems
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftBack"),
                new Motor(hardwareMap, "rightBack")
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        clawSS = new ClawSubsystem(hardwareMap, "claw");
        liftSS = new LiftSubsystem(hardwareMap, "lift");

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .and(driver.getGamepadButton(GamepadKeys.Button.A).negate())
                .toggleWhenActive(new InstantCommand(clawSS::grab,clawSS));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(driver.getGamepadButton(GamepadKeys.Button.Y).negate())
                .toggleWhenActive(new InstantCommand(clawSS::release,clawSS));
        /*
        driver.getGamepadButton(GamepadKeys.Trigger.RIGHT_TRIGGER)
                .and(driver.getGamepadButton(GamepadKeys.Trigger.LEFT_TRIGGER).negate())
                .whenActive(new InstantCommand(liftSS::up,liftSS));

        driver.getGamepadButton(GamepadKeys.Trigger.RIGHT_TRIGGER)
                .and(driver.getGamepadButton(GamepadKeys.Trigger.LEFT_TRIGGER).negate())
                .whenActive(new InstantCommand(liftSS::up,liftSS));
        */
        drive.driveFieldCentric(-driver.getLeftX(),-driver.getLeftY(),-driver.getRightX(), imu.getRotation2d().getDegrees());

    }
}

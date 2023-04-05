package org.firstinspires.ftc.teamcode.old;

/*
@TeleOp(name="TeleOp_Dual")
public class TwoClawTele2 extends CommandOpMode {

    private GamepadEx driver;
    private MecanumDrive mecanumDrive;

    private ColorSubsystem colorSS;

    private DriveSubsystem drive;
    private RevIMU imu;
    // private DualClawSubsystem clawSS;
    private TwoClawSubsystem clawSS;
    private LiftSubsystem liftSS;

    private boolean autoDetect;

    @Override
    public void initialize() {

        // Gamepad(s)
        driver = new GamepadEx(gamepad1);

        // Drive
        drive = new DriveSubsystem(hardwareMap);
        imu = new RevIMU(hardwareMap);
        imu.init();

        // Subsystems
        // clawSS = new DualClawSubsystem(hardwareMap, "claw", "rightClaw");
        clawSS = new TwoClawSubsystem(hardwareMap, "claw", "rightClaw");


        liftSS = new LiftSubsystem(hardwareMap, "lift");

        colorSS = new ColorSubsystem(hardwareMap, "color");

        // claw commands
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new TwoClawGrab(clawSS));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new TwoClawRelease(clawSS));

        // lift commands
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftUp(liftSS))
                .whenReleased(new InstantCommand(liftSS::stop, liftSS));
                // .whenReleased(new InstantCommand(liftSS::hold, liftSS));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new LiftDown(liftSS))
                .whenReleased(new InstantCommand(liftSS::stop, liftSS));
                // .whenReleased(new InstantCommand(liftSS::hold, liftSS));

        drive.setDefaultCommand(new DefDrive(drive, driver::getLeftX, driver::getLeftY, driver::getRightX));
    }
}
*/
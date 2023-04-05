package org.firstinspires.ftc.teamcode.old;

/*
@TeleOp(name="TeleOp_2Claw_Parallel")
public class TwoClawTele_Para extends CommandOpMode {

    private GamepadEx driver;
    private MecanumDrive mecanumDrive;

    private ColorSubsystem colorSS;

    private DriveSubsystem drive;
    // private DualClawSubsystem clawSS;
    private LeftClawSubsystem lClawSS;
    private RightClawSubsystem rClawSS;
    // private TwoClawSubsystem clawSS;
    private LiftSubsystem liftSS;

    private boolean autoDetect;

    @Override
    public void initialize() {

        // Gamepad(s)
        driver = new GamepadEx(gamepad1);

        // Drive
        drive = new DriveSubsystem(hardwareMap);

        // Subsystems
        // clawSS = new DualClawSubsystem(hardwareMap, "claw", "rightClaw");
        // clawSS = new TwoClawSubsystem(hardwareMap, "claw", "rightClaw");

        lClawSS = new LeftClawSubsystem(hardwareMap);
        rClawSS = new RightClawSubsystem(hardwareMap);

        liftSS = new LiftSubsystem(hardwareMap, "lift");

        colorSS = new ColorSubsystem(hardwareMap, "color");

        // claw commands
        driver.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(new ParallelCommandGroup(
                                new InstantCommand(lClawSS::grab, lClawSS),
                                new InstantCommand(rClawSS::grab, rClawSS)
                                )
                        );
                //.whenPressed(new InstantCommand(clawSS::grab, clawSS));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(lClawSS::release, lClawSS),
                        new InstantCommand(rClawSS::release, rClawSS))
                );
                // .whenPressed(new InstantCommand(clawSS::release, clawSS));

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

package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.MoveFB;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@Autonomous(name="Auto Forward")
public class AutoForward extends CommandOpMode {

    static final double COUNTS_PER_MOTOR_REV = 312;    // Gobilda 5202 19.2:1 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = 45;
    static final double DIST_PER_REV = Math.PI * 4.0;

    DriveSubsystem drive;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            schedule(new MoveFB(drive,40, 1));

            stop();
        }

        drive.positionMode();
    }
}

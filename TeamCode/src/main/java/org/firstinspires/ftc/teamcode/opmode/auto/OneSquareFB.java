package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.MoveFB;
import org.firstinspires.ftc.teamcode.commands.MoveLR;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Disabled
@Autonomous(name="1SquareFB")
public class OneSquareFB extends CommandOpMode {

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

        schedule(new SequentialCommandGroup(
                new MoveFB(drive,24, 1),
                new WaitCommand(2000),
                new MoveFB(drive,-24, 1)
        ));

        while (opModeIsActive() && !isStopRequested()) {

            CommandScheduler.getInstance().run();
            // encoder revs = 0.3463747 * dist
            telemetry.addData("Dist / revs: ", 24 / drive.getLeftEncoderRevs());
            telemetry.addData("Revs: ", drive.getLeftEncoderRevs());
            telemetry.update();
        }
    }
}

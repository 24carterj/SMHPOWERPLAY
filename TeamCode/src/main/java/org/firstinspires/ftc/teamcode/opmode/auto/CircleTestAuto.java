package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.MoveFB;
import org.firstinspires.ftc.teamcode.commands.MoveLR;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@Disabled
@Autonomous(name="Square")
public class CircleTestAuto extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 312;    // Gobilda 5202 19.2:1 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = 45;
    static final double DIST_PER_REV = Math.PI * 4.0;

    DriveSubsystem drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveSubsystem(hardwareMap);

        CommandScheduler.getInstance().reset();

        SequentialCommandGroup s = new SequentialCommandGroup(
                new MoveFB(drive,10, 0.7).withTimeout(500),
                new WaitCommand(300),
                new MoveLR(drive, 10, 0.7).withTimeout(500),
                new WaitCommand(300),
                new MoveFB(drive,-10, 0.7).withTimeout(500),
                new WaitCommand(300),
                new MoveLR(drive, -10, 0.7).withTimeout(500),
                new WaitCommand(300),
                new InstantCommand(this::requestOpModeStop)
        );

        // CommandScheduler.getInstance().schedule(new InstantCommand(this::requestOpModeStop));

        waitForStart();
        while (opModeIsActive()) {
            /*
            new MoveFB(drive,10, 0.7).withTimeout(500);
            new WaitCommand(300);
            new MoveLR(drive, 10, 0.7).withTimeout(500);
            new WaitCommand(300);
            new MoveFB(drive,-10, 0.7).withTimeout(500);
            new WaitCommand(300);
            new MoveLR(drive, -10, 0.7).withTimeout(500);
            */
            CommandScheduler.getInstance().schedule(s);
            CommandScheduler.getInstance().run();
            //new InstantCommand(this::requestOpModeStop);
        }
        /*
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            SequentialCommandGroup s = new SequentialCommandGroup(
                    new MoveFB(drive,10, 1).withTimeout(500),
                    new MoveLR(drive, 10, 1).withTimeout(500),
                    new MoveFB(drive,-10, 1).withTimeout(500),
                    new MoveLR(drive, -10, 1).withTimeout(500)
            );
            schedule(s);
            /*
            schedule(new MoveFB(drive,10, 0.5));
            sleep(50);
            schedule(new MoveLR(drive, 10, 0.5));
            sleep(50);
            schedule(new MoveFB(drive,-10, 0.5));
            sleep(50);
            schedule(new MoveLR(drive, -10, 0.5));
            sleep(50);

            r
            if (s.isFinished()) {
                requestOpModeStop();
                stop();
            }
        }
        */
        drive.positionMode();
    }
}

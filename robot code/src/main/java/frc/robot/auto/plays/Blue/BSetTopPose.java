package frc.robot.auto.plays.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BSetTopPose extends SequentialCommandGroup{
    public BSetTopPose(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.83, 4.34, 180),
            new ParallelCommandGroup(

            )

        );
    }
}

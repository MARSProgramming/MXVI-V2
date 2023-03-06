package frc.robot.auto.plays.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BSetMidPose extends SequentialCommandGroup{
    public BSetMidPose(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        addCommands(
            new ZeroGyroscope(drivetrain, 180),
            new ResetDrivePose(drivetrain, 1.83, 2.69, 180)
        );
    }
}

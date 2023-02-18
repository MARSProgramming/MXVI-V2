package frc.robot.auto.plays.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BSetMidPose extends SequentialCommandGroup{
    public BSetMidPose(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 2.69, 0),
            new ParallelCommandGroup(

            )
        );
    }
}

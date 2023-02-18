package frc.robot.auto.plays.Red;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RbP3_NoDock extends SequentialCommandGroup{
    public RbP3_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory MarkertoP3 = AutoChooser.openTrajectoryFile("RED_BottomMarker_M-P3.wpilib.json");
        Trajectory P3toMarker = AutoChooser.openTrajectoryFile("RED_BottomMarker_P3-M.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP3, 0, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P3toMarker, 0, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw to release and score piece
                // Move arm into "default" position
                // Any necessary changes to prepare for Teleop here
            )
        );
    }
}

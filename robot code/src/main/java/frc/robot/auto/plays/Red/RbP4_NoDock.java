package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RbP4_NoDock extends SequentialCommandGroup{
    public RbP4_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory MarkertoP4 = AutoChooser.openTrajectoryFile("RED_BottomMarker_M-P4");
        PathPlannerTrajectory P4toMarker = AutoChooser.openTrajectoryFile("RED_BottomMarker_P4-M");
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP4, 0, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P4toMarker, 0, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
                // Any changes to prepare for Teleop here
            )

        );
    }
}

package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BtP1_NoDock extends SequentialCommandGroup{
    public BtP1_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P1");
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P1-M");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 4.36, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP1, 0, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P1toMarker, 0, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
                // Any changes to prepare for teleop here
            )
        );
    }
}

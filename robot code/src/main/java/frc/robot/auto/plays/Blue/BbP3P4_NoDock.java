package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BbP3P4_NoDock extends SequentialCommandGroup{
    public BbP3P4_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);
        
        PathPlannerTrajectory MarkertoP3 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P3");
        PathPlannerTrajectory P3toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P3-M");
        PathPlannerTrajectory MarkertoP4 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P4");
        PathPlannerTrajectory P4toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P4-M");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 0.45, 0),
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
                // Open claw
                // Move arm into "default" position
            ),
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

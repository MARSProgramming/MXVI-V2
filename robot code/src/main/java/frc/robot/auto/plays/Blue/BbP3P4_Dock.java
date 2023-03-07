//not done

package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BbP3P4_Dock extends SequentialCommandGroup{
    public BbP3P4_Dock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);
        PathPlannerTrajectory MarkertoP3 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P3", new PathConstraints(4, 3));
        PathPlannerTrajectory P3toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P3-M", new PathConstraints(4, 3));
        PathPlannerTrajectory MarkertoP4 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P4", new PathConstraints(4, 3));
        PathPlannerTrajectory P4toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P4-M", new PathConstraints(4, 3));
        PathPlannerTrajectory MarkerToDock = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-C", new PathConstraints(4, 3));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.83, 0.96, 180),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP3, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P3toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP4, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
            new DriveAtPath(drivetrain, P4toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            ),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkerToDock, false, false, 10)
                // Any changes to prepare for Teleop here
            )
        

        );
    }
}

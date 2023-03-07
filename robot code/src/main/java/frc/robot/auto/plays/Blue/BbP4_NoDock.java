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
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BbP4_NoDock extends SequentialCommandGroup{
    public BbP4_NoDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory MarkertoP4 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P4", new PathConstraints(1.5, 0.75));
        PathPlannerTrajectory P4toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P4-M", new PathConstraints(1.5, 0.75));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.83, 0.96, 180),
            mManipulator.goToCubeShootHigh().withTimeout(1).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.5),
            mManipulator.goToZero().withTimeout(1),
            new DriveAtPath(drivetrain, MarkertoP4, false, false, 7).deadlineWith(
                mManipulator.goToHighIntake(),
                mManipulator.getGrasper().runTestMode()
            ),
            new DriveAtPath(drivetrain, P4toMarker, false, false, 7).deadlineWith(
                mManipulator.goToZero()
            )
            /*new ParallelCommandGroup(
                new DriveAtPath(drivetrain, P4toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
                // Any changes to prepare for Teleop here
            )*/

        );
    }
}

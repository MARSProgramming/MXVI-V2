//not done

package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BP4_Cone_Dock extends SequentialCommandGroup{
    public BP4_Cone_Dock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory MarkertoP4 = AutoChooser.openTrajectoryFileForAlliance("BLUE_BottomMarker_M-P4Cone", new PathConstraints(2, 2.0));
        PathPlannerTrajectory P4toMarker = AutoChooser.openTrajectoryFileForAlliance("BLUE_BottomMarker_P4-MCone", new PathConstraints(2, 2.0));
        PathPlannerTrajectory MarkerToDock = AutoChooser.openTrajectoryFileForAlliance("BLUE_BottomMarker_M-CCone", new PathConstraints(2, 1.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.05),
            new ResetDrivePose(drivetrain, MarkertoP4.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(2.7),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            new DriveAtPath(drivetrain, MarkertoP4, false, false, 5).deadlineWith(
                new WaitCommand(1).andThen(mManipulator.goToCloseCubeIntake()),
                mManipulator.getGrasper().runTestMode()
            ),
            new DriveAtPath(drivetrain, P4toMarker, false, false, 3.5).deadlineWith(
                mManipulator.goToZero(),
                mManipulator.getGrasper().runTestCurrent()
            ),
            mManipulator.goToCubeShootHigh().withTimeout(1).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.2),
            new DriveAtPath(drivetrain, MarkerToDock, false, false, 5.5).deadlineWith(
                mManipulator.goToZero()
            ),
            new AutoBalance(drivetrain)
        );
    }
}

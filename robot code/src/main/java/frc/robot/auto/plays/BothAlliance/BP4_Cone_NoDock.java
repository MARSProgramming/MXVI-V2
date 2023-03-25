//not done

package frc.robot.auto.plays.BothAlliance;

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

public class BP4_Cone_NoDock extends SequentialCommandGroup{
    public BP4_Cone_NoDock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory MarkertoP4 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P4Cone", new PathConstraints(1.25, 1));
        PathPlannerTrajectory P4toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P4-MCone", new PathConstraints(1.25, 1));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.05),
            new ResetDrivePose(drivetrain, MarkertoP4.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(3.0),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToCubeIntake().withTimeout(0.2),
            new DriveAtPath(drivetrain, MarkertoP4, false, false, 5).deadlineWith(
                mManipulator.goToCubeIntake(),
                mManipulator.getGrasper().runTestMode()
            ),
            new DriveAtPath(drivetrain, P4toMarker, false, false, 5.5).deadlineWith(
                mManipulator.goToZero(),
                mManipulator.getGrasper().runTestCurrent()
            ),
            mManipulator.goToCubeShootHigh().withTimeout(1).deadlineWith(mManipulator.getGrasper().runTestCurrent()),
            mManipulator.getGrasper().runSpitMode().withTimeout(0.2),
            mManipulator.goToZero()
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

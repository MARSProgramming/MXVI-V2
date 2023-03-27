package frc.robot.auto.plays.BothAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Manipulator;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class TP1_Cone_Dock extends SequentialCommandGroup{
    public TP1_Cone_Dock(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain, mManipulator);

        PathPlannerTrajectory MarkertoP1 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P1Cone", new PathConstraints(3, 2));
        PathPlannerTrajectory P1toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P1-MCone", new PathConstraints(3, 2));
        PathPlannerTrajectory CSPath = AutoChooser.openTrajectoryFile("Blue_TopMarker_M-CCone", new PathConstraints(1, 1));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, MarkertoP1.getInitialHolonomicPose()).withTimeout(0.03),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(2.6),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            new DriveAtPath(drivetrain, MarkertoP1, false, false, 5.0).deadlineWith(
                mManipulator.goToCloseCubeIntake(), mManipulator.getGrasper().setPercentOutputCommand(0.6)
            ),
            new DriveAtPath(drivetrain, P1toMarker, false, false, 2.9).deadlineWith(
                mManipulator.goToZero().withTimeout(0.5).andThen(mManipulator.goToCubeShootHigh()),
                mManipulator.getGrasper().runTestCurrent().withTimeout(3)
            ),
            mManipulator.getGrasper().setPercentOutputCommand(-1).withTimeout(0.3),
            mManipulator.goToZero().withTimeout(2.0).alongWith(
            new DriveAtPath(drivetrain, CSPath, false, false, 3.7)),
            new AutoBalance(drivetrain)
        


                
            /*new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP2, false, false, 10)
                // Code for extending intake
                // Code for retracting intake
                // Move arm to retrieve game piece, and open claw
                // Once in position, close claw
                // Make any readjustments necessary for making sure the piece is secure
            ),
            new ParallelCommandGroup(
            new DriveAtPath(drivetrain, P2toMarker, false, false, 10)
                // Move arm (if necessary) to position game piece for scoring
                // Open claw
                // Move arm into "default" position
            )*/
        );
    }
}

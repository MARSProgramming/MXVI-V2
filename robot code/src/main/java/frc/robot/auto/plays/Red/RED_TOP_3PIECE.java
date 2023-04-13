package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class RED_TOP_3PIECE extends SequentialCommandGroup{
    public RED_TOP_3PIECE(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory P1 = AutoChooser.openTrajectoryFileForRed("RED_TOP_G-P1", new PathConstraints(3, 1.9));
        PathPlannerTrajectory ScoreP1 = AutoChooser.openTrajectoryFileForRed("RED_TOP_P1-G", new PathConstraints(3, 2.5));
        PathPlannerTrajectory P2 = AutoChooser.openTrajectoryFileForRed("RED_TOP_G-P2", new PathConstraints(3, 2));
        PathPlannerTrajectory ScoreP2 = AutoChooser.openTrajectoryFileForRed("RED_TOP_P2-G", new PathConstraints(3, 2));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, P1.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(2.5),
            new DriveAtPath(drivetrain, P1, false, false, 5).deadlineWith(
                mManipulator.goToCloseCubeIntake().alongWith(mManipulator.getGrasper().setPercentOutputCommand(1))
            ),
            new DriveAtPath(drivetrain, ScoreP1, false, false, 3).alongWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() < 3.6).deadlineWith(mManipulator.getGrasper().runTestCurrent()).andThen(mManipulator.goToScoreHigh().withTimeout(2.2))
            ),
            new DriveAtPath(drivetrain, P2, false, false, 5).deadlineWith(
                mManipulator.goToCloseCubeIntake().alongWith(mManipulator.getGrasper().setPercentOutputCommand(1))
            ),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            new DriveAtPath(drivetrain, ScoreP2, false, false, 2.7).deadlineWith(
                mManipulator.goToZero().withTimeout(1).andThen(mManipulator.goToScoreMid()), mManipulator.getGrasper().runTestCurrent()
            ),
            mManipulator.getGrasper().setPercentOutputCommand(-1).withTimeout(0.2),
            mManipulator.goToZero()
        );
    }
}

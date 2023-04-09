package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BLUE_BOT_3PIECE extends SequentialCommandGroup{
    public BLUE_BOT_3PIECE(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory P1 = AutoChooser.openTrajectoryFileForBlue("BLUE_BOT_G-P4", new PathConstraints(2, 2));
        PathPlannerTrajectory ScoreP1 = AutoChooser.openTrajectoryFileForBlue("BLUE_BOT_P4-G", new PathConstraints(2, 2));
        PathPlannerTrajectory P2 = AutoChooser.openTrajectoryFileForBlue("BLUE_BOT_G-P3", new PathConstraints(2, 2));
        PathPlannerTrajectory ScoreP2 = AutoChooser.openTrajectoryFileForBlue("BLUE_BOT_P3-G", new PathConstraints(2, 2));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, P1.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreMid().withTimeout(1.5),
            new DriveAtPath(drivetrain, P1, false, false, 5).deadlineWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() > 2.6).andThen(mManipulator.goToCloseCubeIntake().alongWith(mManipulator.getGrasper().setPercentOutputCommand(1)))
            ),
            new DriveAtPath(drivetrain, ScoreP1, false, false, 3).alongWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() < 4.2).andThen(mManipulator.goToCubeShootHigh().withTimeout(1.5)).deadlineWith(mManipulator.getGrasper().runTestCurrent())
            ),
            mManipulator.getGrasper().setPercentOutputCommand(-1).withTimeout(0.2),
            new DriveAtPath(drivetrain, P2, false, false, 5).deadlineWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() > 3.6).andThen(mManipulator.goToCloseCubeIntake().alongWith(mManipulator.getGrasper().setPercentOutputCommand(1)))
            ),
            new DriveAtPath(drivetrain, ScoreP2, false, false, 4).alongWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() < 3.6).deadlineWith(mManipulator.getGrasper().runTestCurrent()).andThen(mManipulator.goToScoreMid().withTimeout(1.5))
            ),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToZero()
        );
    }
}

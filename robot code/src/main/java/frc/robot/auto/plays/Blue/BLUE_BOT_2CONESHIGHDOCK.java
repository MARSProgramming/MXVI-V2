package frc.robot.auto.plays.Blue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.AutoChooser;

public class BLUE_BOT_2CONESHIGHDOCK extends SequentialCommandGroup{
    public BLUE_BOT_2CONESHIGHDOCK(DrivetrainSubsystem drivetrain, Manipulator mManipulator){
        addRequirements(drivetrain);

        PathPlannerTrajectory P1 = AutoChooser.openTrajectoryFileForBlue("BLUE_BOT_G-P4", new PathConstraints(2, 1.5));
        PathPlannerTrajectory ScoreP1 = AutoChooser.openTrajectoryFileForBlue("BLUE_BOT_P4-G", new PathConstraints(2, 1.5));
        PathPlannerTrajectory Dock = AutoChooser.openTrajectoryFileForBlue("BLUE_BOT_G-C", new PathConstraints(1.5, 1.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.03),
            new ResetDrivePose(drivetrain, P1.getInitialHolonomicPose()),
            mManipulator.swapAutoScoreCommand().withTimeout(0.03),
            mManipulator.goToScoreHigh().withTimeout(2.5),
            new DriveAtPath(drivetrain, P1, false, false, 3.5).deadlineWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() > 2.6).andThen(mManipulator.goToCloseCubeIntake().alongWith(mManipulator.getGrasper().setPercentOutputCommand(1)))
            ),
            new DriveAtPath(drivetrain, ScoreP1, false, false, 5).alongWith(
                mManipulator.goToZero().until(() -> drivetrain.getPose().getX() < 3.6).andThen(mManipulator.goToScoreHigh().withTimeout(2.5)).deadlineWith(mManipulator.getGrasper().runTestCurrent())
            ),
            new DriveAtPath(drivetrain, Dock, false, false, 4).deadlineWith(mManipulator.goToZero()),
            new AutoBalance(drivetrain).alongWith(mManipulator.swapAutoScoreCommand().withTimeout(0.03))
        );
    }
}

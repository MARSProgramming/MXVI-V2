package frc.robot.auto.plays.Red;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DriveAtPath;
import frc.robot.commands.Drive.ResetDrivePose;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RED_MID_LEAVEDOCK extends SequentialCommandGroup{
    public RED_MID_LEAVEDOCK(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        PathPlannerTrajectory LeaveCommunity = AutoChooser.openTrajectoryFileForRed("RED_MID_LEAVE", new PathConstraints(1.0, 0.5));
        PathPlannerTrajectory Dock = AutoChooser.openTrajectoryFileForRed("RED_MID_LEAVE-DOCK", new PathConstraints(1.0, 0.5));
        addCommands(
            new ZeroGyroscope(drivetrain, 180).withTimeout(0.1),
            new ResetDrivePose(drivetrain, LeaveCommunity.getInitialHolonomicPose()),
            new DriveAtPath(drivetrain, LeaveCommunity, false, false, 7),
            new DriveAtPath(drivetrain, Dock, false, false, 3),
            new AutoBalance(drivetrain)
        );
    }
}

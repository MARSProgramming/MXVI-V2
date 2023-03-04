
package frc.robot.commands.Drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveAtPath extends CommandBase {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private PathPlannerTrajectory mTrajectory;
    private HolonomicDriveController mController;
    private Timer mTimer;
    private double timeout;
    private boolean balancePos;
    private boolean balanceStatus;

    public DriveAtPath(DrivetrainSubsystem subsystem, PathPlannerTrajectory traj, boolean balance, boolean balanceClose, double timeout) {
        mTrajectory = traj;
        mDrivetrainSubsystem = subsystem;
        mController = subsystem.getDrivePathController();
        balanceStatus = balance;
        mTimer = new Timer();
        this.timeout = timeout;
        balancePos = balanceClose;
        mController.setTolerance(new Pose2d(0.03, 0.03, new Rotation2d(0.05)));
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mTimer.reset();
        mTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var state = (PathPlannerState) mTrajectory.sample(mTimer.get() + 0.3);
        mDrivetrainSubsystem.drive(mController.calculate(mDrivetrainSubsystem.getPose(), mTrajectory.sample(mTimer.get() + 0.3), state.holonomicRotation));
        SmartDashboard.putNumber("desiredX", mTrajectory.sample(mTimer.get() + 0.3).poseMeters.getX());
        SmartDashboard.putNumber("autoXError", mDrivetrainSubsystem.getPose().getX() - mTrajectory.sample(mTimer.get() + 0.3).poseMeters.getX());
        SmartDashboard.putNumber("desiredY", mTrajectory.sample(mTimer.get() + 0.3).poseMeters.getY());
        SmartDashboard.putNumber("autoYError", mDrivetrainSubsystem.getPose().getY() - mTrajectory.sample(mTimer.get() + 0.3).poseMeters.getY());
        SmartDashboard.putNumber("desiredrot", state.holonomicRotation.getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDrivetrainSubsystem.drive(new ChassisSpeeds());
        mTimer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(mTrajectory.getEndState().poseMeters.getTranslation()) < 0.03
        || mTimer.get() > timeout
        || (balanceStatus && balancePos && mDrivetrainSubsystem.finishedBalanceClose())
        || (balanceStatus && !balancePos && mDrivetrainSubsystem.finishedBalanceFar());
    }
}

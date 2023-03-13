
package frc.robot.commands.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignToScore extends CommandBase {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private PathPlannerTrajectory mTrajectory;
    private HolonomicDriveController mController;
    private Timer mTimer;
    private AlignToScoreEnum mPos = AlignToScoreEnum.LEFT;
    private double align = 0;

    public AlignToScore(DrivetrainSubsystem subsystem, AlignToScoreEnum pos) {
        mPos = pos;
        mDrivetrainSubsystem = subsystem;
        mController = subsystem.getDrivePathController();
        mTimer = new Timer();
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mController.setTolerance(new Pose2d(0.01, 0.01, new Rotation2d(0.004)));
        double y = 0;
        mDrivetrainSubsystem.setAlignAdjust(0);
        if(mPos == AlignToScoreEnum.LEFT){
            y = mDrivetrainSubsystem.getAlignLeftY();
        }
        else if(mPos == AlignToScoreEnum.MID){
            y = mDrivetrainSubsystem.getAlignMidY();
        }
        else if(mPos == AlignToScoreEnum.RIGHT){
            y = mDrivetrainSubsystem.getAlignRightY();    
        }

        mTrajectory = PathPlanner.generatePath(
      new PathConstraints(0.5, 1.5), 
      new PathPoint(mDrivetrainSubsystem.getPose().getTranslation(), new Rotation2d(Math.PI/2), mDrivetrainSubsystem.getGyroscopeRotation()),
      new PathPoint(new Translation2d(1.95, y), new Rotation2d(Math.PI/2), new Rotation2d(Math.PI))
        );
        mTimer.reset();
        mTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var state = (PathPlannerState) mTrajectory.sample(mTimer.get() + 0.3);
        if(align != mDrivetrainSubsystem.getAdjust().getAsDouble()){
            mTrajectory.transformBy(new Transform2d(new Translation2d(0, mDrivetrainSubsystem.getAdjust().getAsDouble()), new Rotation2d()));
            align = mDrivetrainSubsystem.getAdjust().getAsDouble();
        }
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
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(mTrajectory.getEndState().poseMeters.getTranslation()) < 0.01;
    }
}

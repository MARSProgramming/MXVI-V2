
package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignToScore extends CommandBase {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private static ProfiledPIDController xController = new ProfiledPIDController(Constants.Auto.holonomicXkP, 0, 0, new TrapezoidProfile.Constraints(1, 0.7));
    private static ProfiledPIDController yController = new ProfiledPIDController(Constants.Auto.holonomicYkP, 0, 0, new TrapezoidProfile.Constraints(1, 0.7));
    private ProfiledPIDController thetaController;
    private Timer mTimer;
    private AlignToScoreEnum mPos = AlignToScoreEnum.LEFT;
    private double yGoal = 0;
    private double xGoal = 1.85;

    public AlignToScore(DrivetrainSubsystem subsystem, AlignToScoreEnum pos) {
        mPos = pos;
        mDrivetrainSubsystem = subsystem;
        mTimer = new Timer();
        thetaController = subsystem.getSnapController();
        yController.setTolerance(0.01);
        xController.setTolerance(0.01);
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDrivetrainSubsystem.setAlignAdjust(0);
        if(mPos == AlignToScoreEnum.LEFT){
            yGoal = mDrivetrainSubsystem.getAlignLeftY();
        }
        else if(mPos == AlignToScoreEnum.MID){
            yGoal = mDrivetrainSubsystem.getAlignMidY();
        }
        else if(mPos == AlignToScoreEnum.RIGHT){
            yGoal = mDrivetrainSubsystem.getAlignRightY();    
        }

        xController.reset(mDrivetrainSubsystem.getPose().getX(), -mDrivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond);
        yController.reset(mDrivetrainSubsystem.getPose().getY(), -mDrivetrainSubsystem.getChassisSpeeds().vyMetersPerSecond);
        mTimer.reset();
        mTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ChassisSpeeds cs = new ChassisSpeeds(
            -xController.calculate(mDrivetrainSubsystem.getPose().getX(), xGoal),
            -yController.calculate(mDrivetrainSubsystem.getPose().getY(), yGoal + mDrivetrainSubsystem.getAdjust().getAsDouble()),
            thetaController.calculate(mDrivetrainSubsystem.getPigeonAngle(), Math.PI));
        mDrivetrainSubsystem.drive(cs);
        System.out.println("xgoal: " + xGoal + "   ygoal: " + yGoal);
        System.out.println("x:" + mDrivetrainSubsystem.getPose().getX() + "y:" + mDrivetrainSubsystem.getPose().getY());
        System.out.println(yController.calculate(mDrivetrainSubsystem.getPose().getY(), yGoal + mDrivetrainSubsystem.getAdjust().getAsDouble()));
        System.out.println(yController.getPositionError());
        System.out.println(yController.getSetpoint().position);
        System.out.println(thetaController.calculate(mDrivetrainSubsystem.getPigeonAngle(), Math.PI));
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
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(new Translation2d(xGoal, yGoal)) < 0.01;
    }
}

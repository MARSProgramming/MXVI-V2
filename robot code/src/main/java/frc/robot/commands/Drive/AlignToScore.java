
package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;

public class AlignToScore extends CommandBase {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private static ProfiledPIDController xController = new ProfiledPIDController(7.0, 0, 0, new TrapezoidProfile.Constraints(2, 1.5));
    private static ProfiledPIDController yController = new ProfiledPIDController(6.5, 0.5, 0, new TrapezoidProfile.Constraints(2, 1.5));
    private ProfiledPIDController thetaController;
    private Timer mTimer;
    private AlignToScoreEnum mPos = AlignToScoreEnum.LEFT;
    private double yGoal = 0;
    private double xGoal = 1.84;
    private double highScoreX = 0.37;
    private double midScoreX = 0.79;

    public AlignToScore(DrivetrainSubsystem subsystem, AlignToScoreEnum pos) {
        mPos = pos;
        mDrivetrainSubsystem = subsystem;
        mTimer = new Timer();
        thetaController = subsystem.getSnapController();
        yController.setTolerance(0.001);
        xController.setTolerance(0.001);
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //mDrivetrainSubsystem.setAlignAdjust(0);
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
        thetaController.reset(mDrivetrainSubsystem.getPigeonAngle(), 0);
        thetaController.setTolerance(0.01);
        mTimer.reset();
        mTimer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //ChassisSpeeds cs = new ChassisSpeeds();
        double angleAdjust = Math.atan((mDrivetrainSubsystem.getPose().getY()-yGoal)/(mDrivetrainSubsystem.getPose().getX() - midScoreX));
        //if(mDrivetrainSubsystem.getPose().getTranslation().getDistance(new Translation2d(xGoal, yGoal)) > 0.04){
            ChassisSpeeds cs = new ChassisSpeeds(
            -xController.calculate(mDrivetrainSubsystem.getPose().getX(), xGoal),
            -yController.calculate(mDrivetrainSubsystem.getPose().getY(), yGoal + mDrivetrainSubsystem.getAdjust().getAsDouble()),
            thetaController.calculate(mDrivetrainSubsystem.getPigeonAngle(), Math.PI));
        //}
        /*else{
            cs = new ChassisSpeeds(0, 0, thetaController.calculate(mDrivetrainSubsystem.getPigeonAngle(), Math.PI + angleAdjust));
        }*/
        
        mDrivetrainSubsystem.drive(cs);
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
        return thetaController.atSetpoint() && mDrivetrainSubsystem.getPose().getTranslation().getDistance(new Translation2d(xGoal, yGoal)) < 0.04 || mDrivetrainSubsystem.getPose().getX() > 3.6 || mDrivetrainSubsystem.getPose().getX() < 1.7;
    }
}

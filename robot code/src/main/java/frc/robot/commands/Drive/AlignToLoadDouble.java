
package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LED;

public class AlignToLoadDouble extends CommandBase {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private double maxSpeedX = 1;
    private double kP = 2.0;
    private ProfiledPIDController thetaController;
    private LED mLED;

    public AlignToLoadDouble(DrivetrainSubsystem subsystem, LED led) {
        mLED = led;
        mDrivetrainSubsystem = subsystem;
        thetaController = subsystem.getSnapController();
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        thetaController.reset(mDrivetrainSubsystem.getPigeonAngle(), 0);
        thetaController.setTolerance(0.01);
        mLED.setFlashing(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ChassisSpeeds cs = new ChassisSpeeds(Math.min(maxSpeedX, kP * (NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2])), 0, thetaController.calculate(mDrivetrainSubsystem.getPigeonAngle(), 0));
        mDrivetrainSubsystem.drive(cs);
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDrivetrainSubsystem.drive(new ChassisSpeeds());
        mLED.setFlashing(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return kP * (NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2]) < 0.3;
    }
}

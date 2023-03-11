package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignWithRRTape extends CommandBase{

    private DrivetrainSubsystem mDrivetrainSubsystem; 
    private PIDController xController;
    private DoubleSupplier mOffset;
    public AlignWithRRTape(DrivetrainSubsystem dt, DoubleSupplier offset){
        mDrivetrainSubsystem = dt;
        mOffset = offset;
        xController = new PIDController(0.03, 0, 0);
    }
    
    @Override
    public void initialize(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    }

    @Override
    public void execute(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry tv = table.getEntry("tv");
        if(tv.getDouble(0) == 1){
            double ySpeed = MathUtil.clamp(Constants.Drive.RRTapeAlignP * tx.getDouble(0), -0.5 , 0.5);
            double xSpeed = MathUtil.clamp(xController.calculate(mDrivetrainSubsystem.getPose().getX(), 1.86), -0.5 , 0.5);
            mDrivetrainSubsystem.drive(new ChassisSpeeds(xSpeed, ySpeed, 0));
        }
    }

    @Override
    public void end(boolean interrupted){
        mDrivetrainSubsystem.drive(new ChassisSpeeds());
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }
}

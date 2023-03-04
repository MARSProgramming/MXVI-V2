package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    double[] botpose;
    private DrivetrainSubsystem dt;
    public Limelight(DrivetrainSubsystem drive){
        dt = drive;
    }

    public void resetPose(){
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1.0 && dt.getPose().getX() < 2.6){
            botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            double x = botpose[0];
            double y = botpose[1];
            double z = botpose[2];
            dt.addVisionMeasurement(new Pose2d(x, y, dt.getGyroscopeRotation()));
        }
    }
}

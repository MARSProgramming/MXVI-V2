// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MoreMath;
import io.github.oblarg.oblog.Loggable;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable{
  private static DrivetrainSubsystem mInstance;
  public static DrivetrainSubsystem getInstance(){
        if (mInstance == null) mInstance = new DrivetrainSubsystem();
        return mInstance;
  } 


  public static final double MAX_VOLTAGE = 12.0;
  private ProfiledPIDController mSnapController;
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI * Constants.Drive.MAX_SPEED_MULTIPLIER;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Constants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Constants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(Constants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-Constants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-Constants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );


  public void zeroSwerves(boolean run) {
   if(run){
        File swerveZeros = new File("/home/lvuser/constants/SwerveZeros.txt");
        swerveZeros.setExecutable(true);
        swerveZeros.setReadable(true);
        swerveZeros.setWritable(true);
        System.out.println("/home/lvuser/constants/SwerveZeros.txt");
        try{
            swerveZeros.createNewFile();
            FileWriter writer = new FileWriter("/home/lvuser/constants/SwerveZeros.txt");
            writer.write(MoreMath.floorMod(Math.toDegrees(m_frontLeftModule.getSteerAngle()-Constants.Drive.FRONT_LEFT_MODULE_STEER_OFFSET), 360) + "\n");
            writer.write(MoreMath.floorMod(Math.toDegrees(m_frontRightModule.getSteerAngle()-Constants.Drive.FRONT_RIGHT_MODULE_STEER_OFFSET), 360) + "\n");
            writer.write(MoreMath.floorMod(Math.toDegrees(m_backLeftModule.getSteerAngle()-Constants.Drive.BACK_LEFT_MODULE_STEER_OFFSET), 360) + "\n");
            writer.write(MoreMath.floorMod(Math.toDegrees(m_backRightModule.getSteerAngle()-Constants.Drive.BACK_RIGHT_MODULE_STEER_OFFSET), 360) + "\n");
            writer.close();
        }
        catch(IOException e){
            System.out.println("File could not be found when writing to swerve zeros");
        }
    }
  }

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.Drive.DRIVETRAIN_PIGEON_ID);
  
  private double pigeonYawOffset = 0.0;
  public double getPigeonAngle(){
        return Math.toRadians(m_pigeon.getYaw());
  }
  public ProfiledPIDController getSnapController(){
          return mSnapController;
  }
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(2.0, 2.0, 2.0);

  private final SwerveDrivePoseEstimator mPoseEstimator;

  private SwerveModule m_frontLeftModule;
  private SwerveModule m_frontRightModule;
  private SwerveModule m_backLeftModule;
  private SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private HolonomicDriveController mHolonomicDriveController = new HolonomicDriveController(new PIDController(Constants.Auto.holonomicXkP, Constants.Auto.holonomicXkI, Constants.Auto.holonomicXkD), new PIDController(Constants.Auto.holonomicYkP, Constants.Auto.holonomicYkI, Constants.Auto.holonomicYkD), new ProfiledPIDController(Constants.Auto.holonomicOkP, Constants.Auto.holonomicOkI, Constants.Auto.holonomicOkD, new TrapezoidProfile.Constraints(Constants.Auto.holonomicOMaxVelocity, Constants.Auto.holonomicOMaxAcceleration)));
  public DrivetrainSubsystem() {
        double fr = 0;
        double fl = 0;
        double br = 0;
        double bl = 0;
        File swerveZeros = new File("/home/lvuser/constants/SwerveZeros.txt");
            if (swerveZeros.exists()) {
                try{
                    Scanner sc = new Scanner(swerveZeros);
                    fl = -Math.toRadians(Double.parseDouble(sc.nextLine()));
                    fr = -Math.toRadians(Double.parseDouble(sc.nextLine()));
                    bl = -Math.toRadians(Double.parseDouble(sc.nextLine()));
                    br = -Math.toRadians(Double.parseDouble(sc.nextLine())); 
                    sc.close();
                    System.out.println(fl);
                    System.out.println(fr);
                    System.out.println(bl);
                    System.out.println(br);
                }
                catch(FileNotFoundException e){
                    System.out.println("Swerve Zeros file not found");
                }
            }
            else{
                fr = Constants.Drive.FRONT_RIGHT_MODULE_STEER_OFFSET;
                fl = Constants.Drive.FRONT_LEFT_MODULE_STEER_OFFSET;
                br = Constants.Drive.BACK_RIGHT_MODULE_STEER_OFFSET;
                bl = Constants.Drive.BACK_LEFT_MODULE_STEER_OFFSET;
            }

    createSwerveModules(fl, fr, bl, br);

    mSnapController = new ProfiledPIDController(Constants.Drive.kP,
        Constants.Drive.kI, 
        Constants.Drive.kD,
        new TrapezoidProfile.Constraints(Constants.Auto.holonomicOMaxVelocity, Constants.Auto.holonomicOMaxAcceleration));
        mSnapController.enableContinuousInput(-Math.PI, Math.PI);
    mPoseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics, 
        new Rotation2d(m_pigeon.getYaw()),
        getSwerveModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
  }
  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */

   public HolonomicDriveController getDrivePathController(){
        return mHolonomicDriveController;
   }
  public void zeroGyroscope(double d) {
        m_pigeon.setYaw(d);
        System.out.print("Zeroed!");
      }
  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }
  public void drive(ChassisSpeeds chassisSpeeds) {
        SmartDashboard.putNumber("speeds", chassisSpeeds.vxMetersPerSecond);
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    double[] botpose;
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false)){
        botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[7]);
        SmartDashboard.putNumber("LL latency", botpose[6]);
        mPoseEstimator.addVisionMeasurement(
                new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[3], botpose[4])),
                Timer.getFPGATimestamp()-botpose[6]);
    }

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    
    mPoseEstimator.update(getGyroscopeRotation(), getSwerveModulePositions());
    SmartDashboard.putNumber("X", this.getPose().getX());
    SmartDashboard.putNumber("Y", this.getPose().getY());
    SmartDashboard.putNumber("rot", this.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("pigeon", this.getPigeonAngle());
}
  public Pose2d getPose(){
    return mPoseEstimator.getEstimatedPosition();
  }
  public void setPose(Pose2d pose, Rotation2d rotation){
   mPoseEstimator.resetPosition(rotation, getSwerveModulePositions(), pose);
  }
  public SwerveDriveKinematics getSwerveKinematics(){
        return m_kinematics;
  }

  public SwerveModulePosition[] getSwerveModulePositions(){
        return new SwerveModulePosition[] {
                new SwerveModulePosition(m_frontLeftModule.getPosition(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_frontRightModule.getPosition(), new Rotation2d(m_frontRightModule.getSteerAngle())),
                new SwerveModulePosition(m_backLeftModule.getPosition(), new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_backRightModule.getPosition(), new Rotation2d(m_backRightModule.getSteerAngle()))
        };
  }

  public void createSwerveModules(double fl, double fr, double bl, double br){
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    Mk4ModuleConfiguration test = new Mk4ModuleConfiguration();
    test.setCanivoreName(Constants.Drive.kDriveCANivore);

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            test,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            15,
            Constants.Drive.FRONT_LEFT_MODULE_STEER_MOTOR,
            Constants.Drive.FRONT_LEFT_MODULE_STEER_ENCODER,
            fl
    );

    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            test,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            14,
            Constants.Drive.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.Drive.FRONT_RIGHT_MODULE_STEER_ENCODER,
            fr
    );
    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            test,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            16,
            Constants.Drive.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.Drive.BACK_LEFT_MODULE_STEER_ENCODER,
            bl
    );
    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            test,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            17, 
            Constants.Drive.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.Drive.BACK_RIGHT_MODULE_STEER_ENCODER,
            br
    );
  }
}

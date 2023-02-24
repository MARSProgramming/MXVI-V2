// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.commands.Drive.ZeroSwerves;
import frc.robot.subsystems.BottomSolenoids;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MiniSystems.Elevator;
import frc.robot.subsystems.MiniSystems.Grasper;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.subsystems.MiniSystems.Wrist;
import frc.robot.util.AutoChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();

  private final CommandXboxController mPilot = new CommandXboxController(0);

  private HashMap<String, Pose2d> mPointPositionMap;
  private AutoChooser autoChooser = new AutoChooser(mDrivetrainSubsystem);

  private final BottomSolenoids mBottomSolenoids = new BottomSolenoids();
  private final Elevator mElevator = new Elevator();
  private final Pivot mPivot = new Pivot();
  private final Wrist mWrist = new Wrist();
  private final Grasper mGrasper = new Grasper();
  //private final Manipulator mManipulator = new Manipulator();

  private final Compressor mCompressor = new Compressor(61, PneumaticsModuleType.REVPH);

  public double getPressure(){
    return mCompressor.getPressure();
  }

  public void startCompressor(){
    mCompressor.enableAnalog(100, 110);
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /*mDrivetrainSubsystem.setDefaultCommand(new DriveSnapRotation(
            mDrivetrainSubsystem,
            () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightY()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            mDrivetrainSubsystem.getSnapController()
    ));*/
    mDrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            mDrivetrainSubsystem,
            () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    SmartDashboard.putData("Zero Swerves", new ZeroSwerves(mDrivetrainSubsystem).withTimeout(1).ignoringDisable(true));
    SmartDashboard.putData(CommandScheduler.getInstance());
    mPointPositionMap = new HashMap<>();
    mPointPositionMap.put("A", new Pose2d(0, 0, new Rotation2d(Math.toRadians(0.0))));
    configureTeleopBindings();
  }

  public void initializeSolenoids(){
    mBottomSolenoids.initializeSolenoid();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureTeleopBindings() {
    mPilot.y().onTrue(new ZeroGyroscope(mDrivetrainSubsystem, 0));
    System.out.println("Teleop Bindings Configured");
  }

  public void configureTestBindings(){
    mPilot.leftTrigger(0.2).whileTrue(mElevator.runTestMode(() -> -mPilot.getLeftTriggerAxis()));
    mPilot.rightTrigger(0.2).whileTrue(mElevator.runTestMode(() -> mPilot.getRightTriggerAxis()));
    mPilot.y().whileTrue(mElevator.testSetpoint());
    mPilot.povUp().whileTrue(mGrasper.runTestMode(() -> 0.5));
    mPilot.povDown().whileTrue(mGrasper.runTestMode(() -> -0.5));
    /*mPilot.leftBumper().whileTrue(mPivot.runTestMode(() -> -0.2));
    mPilot.rightBumper().whileTrue(mPivot.runTestMode(() -> 0.2));
    mPilot.povUp().whileTrue(mWrist.runTestMode(() -> 0.2));
    mPilot.povDown().whileTrue(mWrist.runTestMode(() -> -0.2));*/

    System.out.println("Test Bindings Configured");
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);

      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    //"Stage" mode
    //value = Math.round(value * 5.0)/5.0;

    return value;
  }
}

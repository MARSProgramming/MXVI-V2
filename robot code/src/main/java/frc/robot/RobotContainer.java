// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive.AlignToLoadDouble;
import frc.robot.commands.Drive.AlignToScore;
import frc.robot.commands.Drive.AlignToScoreEnum;
import frc.robot.commands.Drive.AutoAlignAdjust;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.ZeroGyroscope;
import frc.robot.commands.Drive.ZeroSwerves;
import frc.robot.commands.Manipulator.AlignToScoreManipulator;
import frc.robot.commands.Manipulator.Elevator.ElevatorScoreMid;
import frc.robot.commands.Manipulator.Grasper.RunIntakeUntilStall;
import frc.robot.commands.Manipulator.Pivot.PivotToIntake;
import frc.robot.commands.Manipulator.Pivot.ZeroPivot;
import frc.robot.subsystems.BottomSolenoids;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SubsystemIO;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.util.AutoChooser;
import frc.robot.util.MatchTab;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();

  private CommandXboxController mPilot = new CommandXboxController(0);
  private CommandXboxController mCopilot = new CommandXboxController(1);
  private CommandXboxController mTestCtrl = new CommandXboxController(2);
  private CommandJoystick mKeypad = new CommandJoystick(3);

  private final BottomSolenoids mBottomSolenoids = new BottomSolenoids();
  private final Limelight mLimelight = new Limelight(mDrivetrainSubsystem);
  private final Manipulator mManipulator = new Manipulator();
  private final SubsystemIO mSubsystemIO = new SubsystemIO();
  private MatchTab matchtab;
  private AutoChooser autoChooser = new AutoChooser(mDrivetrainSubsystem, mManipulator);
  private LED mLED = new LED();

  private final Compressor mCompressor = new Compressor(61, PneumaticsModuleType.REVPH);

  public double getPressure(){
    return mCompressor.getPressure();
  }
  public void resetPose(){
    mLimelight.resetPose();
  }
  public void startCompressor(){
    mCompressor.enableAnalog(100, 110);
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    matchtab = new MatchTab(mDrivetrainSubsystem, mManipulator.getElevator(), mManipulator.getGrasper(), mManipulator.getPivot(), mManipulator.getWrist(), mManipulator);
    matchtab.configureDashboard();
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
    SmartDashboard.putData("Zero Pivot", new ZeroPivot(mManipulator.getPivot()).withTimeout(1).ignoringDisable(true));

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
    mKeypad.button(3).onTrue(mManipulator.setAlignTypeCommand(2));
    mKeypad.button(2).onTrue(mManipulator.setAlignTypeCommand(1));
    mKeypad.button(1).onTrue(mManipulator.setAlignTypeCommand(0));

    mPilot.y().whileTrue(new ZeroGyroscope(mDrivetrainSubsystem, 0));
    mPilot.x().whileTrue((new AlignToScore(mDrivetrainSubsystem, AlignToScoreEnum.LEFT).deadlineWith(mManipulator.getGrasper().runTestCurrent()).andThen(mManipulator.getGrasper().runSpitMode().unless(() -> !mManipulator.getAutoScore()))).alongWith(new AlignToScoreManipulator(mManipulator)));
    mPilot.a().whileTrue((new AlignToScore(mDrivetrainSubsystem, AlignToScoreEnum.MID).deadlineWith(mManipulator.getGrasper().runTestCurrent()).andThen(mManipulator.getGrasper().runSpitMode().unless(() -> !mManipulator.getAutoScore()))).alongWith(new AlignToScoreManipulator(mManipulator)));
    mPilot.b().whileTrue((new AlignToScore(mDrivetrainSubsystem, AlignToScoreEnum.RIGHT).deadlineWith(mManipulator.getGrasper().runTestCurrent()).andThen(mManipulator.getGrasper().runSpitMode().unless(() -> !mManipulator.getAutoScore()))).alongWith(new AlignToScoreManipulator(mManipulator)));
    //mPilot.leftTrigger().whileTrue(new RunIntakeUntilStall(mManipulator).andThen(mManipulator.getGrasper().runTestCurrent()));
    mPilot.leftTrigger().whileTrue(mManipulator.getGrasper().runTestMode());
    mPilot.rightBumper().toggleOnTrue(mManipulator.getGrasper().runTestCurrent());
    mPilot.rightTrigger().whileTrue(mManipulator.getGrasper().runSpitMode().alongWith(mDrivetrainSubsystem.resetAlign()));
    mPilot.leftBumper().onTrue(mLED.swapYellowPurple().andThen(mLED.setFlashingCommand(true).withTimeout(1)).andThen(mLED.setFlashingCommand(false).withTimeout(0.05)));
    mPilot.rightStick().onTrue(new AlignToLoadDouble(mDrivetrainSubsystem, mLED));
    mPilot.povDown().whileTrue(mManipulator.goToStow());
    mPilot.start().onTrue(mBottomSolenoids.toggleBottomSolenoid());

    mPilot.povLeft().whileTrue(new AutoAlignAdjust(mDrivetrainSubsystem, 0.05));
    mPilot.povRight().whileTrue(new AutoAlignAdjust(mDrivetrainSubsystem, -0.05));

    mCopilot.a().whileTrue(mManipulator.goToLoadDouble());
    mCopilot.b().whileTrue(mManipulator.goToCloseCubeIntake());
    mCopilot.y().whileTrue(mManipulator.goToHighIntake());
    mCopilot.x().whileTrue(mManipulator.goToIntake());

    mCopilot.povUp().whileTrue(mManipulator.goToScoreHigh());
    mCopilot.povDown().whileTrue(mManipulator.goToLoadCommand());
    mCopilot.povRight().whileTrue(mManipulator.goToScoreLow());

    new Trigger(() -> mCopilot.getLeftX() > 0.7).whileTrue(mManipulator.getWrist().runTestMode(() -> 0.2));
    new Trigger(() -> mCopilot.getLeftX() < -0.7).whileTrue(mManipulator.getWrist().runTestMode(() -> -0.2));
    new Trigger(() -> mCopilot.getLeftY() < -0.7).whileTrue(mManipulator.getElevator().runTestMode(() -> 0.45));
    new Trigger(() -> mCopilot.getLeftY() > 0.7).whileTrue(mManipulator.getElevator().runTestMode(() -> -0.45));

    mCopilot.rightTrigger().whileTrue(mManipulator.goToZero());
    mCopilot.back().onTrue(mManipulator.swapAutoScoreCommand());
    mCopilot.leftStick().onTrue(mManipulator.goToShoot());
    mCopilot.rightStick().onTrue(mManipulator.goToCubeShootHigh());
    mCopilot.povLeft().whileTrue(mManipulator.goToScoreMid());
    mCopilot.leftBumper().whileTrue(mManipulator.getPivot().runTestMode(() -> -0.4));
    mCopilot.rightBumper().whileTrue(mManipulator.getPivot().runTestMode(() -> 0.4));
    System.out.println("Teleop Bindings Configured");
  }

  public void configureTestBindings(){
    mTestCtrl.leftTrigger(0.1).whileTrue(mManipulator.getElevator().runTestMode(() -> -mTestCtrl.getLeftTriggerAxis()));
    mTestCtrl.rightTrigger(0.1).whileTrue(mManipulator.getElevator().runTestMode(() -> mTestCtrl.getRightTriggerAxis()));
    mTestCtrl.povUp().whileTrue(mManipulator.getGrasper().runTestMode());
    mTestCtrl.povDown().whileTrue(mManipulator.getGrasper().runSpitMode());
    mTestCtrl.leftBumper().whileTrue(mManipulator.getPivot().runTestMode(() -> -0.2));
    mTestCtrl.rightBumper().whileTrue(mManipulator.getPivot().runTestMode(() -> 0.2));
    mTestCtrl.povRight().whileTrue(mManipulator.getWrist().runTestMode(() -> 0.2));
    mTestCtrl.povLeft().whileTrue(mManipulator.getWrist().runTestMode(() -> -0.2));
    mTestCtrl.start().whileTrue(mManipulator.getElevator().disableSoftLimits());
    mTestCtrl.a().whileTrue(mManipulator.getWrist().testIntake());
    mTestCtrl.x().whileTrue(new ElevatorScoreMid(mManipulator));
    mTestCtrl.y().whileTrue(new PivotToIntake(mManipulator));
    mTestCtrl.b().whileTrue(mManipulator.goToZero());


    System.out.println("Test Bindings Configured");
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected().andThen(mLED.setRed());
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
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    //"Stage" mode
    //value = Math.round(value * 5.0)/5.0;

    return value;
    
  }
}

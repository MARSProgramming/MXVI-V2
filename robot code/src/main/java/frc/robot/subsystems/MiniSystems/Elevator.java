package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * Double stage slanted elevator moving the manipulator up and down
 * 
 * 2 Motors - IDs x, y
 * Gear Ratio - 49:1(tentative)
 * Constraints - Cannot move too far up or down
 * 
 * Controls:
 * position control through moveToPosition() method
 * manual movement through movePercentOutput() method
 */

public class Elevator extends SubsystemBase{

    private TalonFX master;
    private TalonFX follower;
    private final double kGearRatio = 35;
    private final double inchesPerRotation = 2 * Math.PI;
    private final double inchesToNativeUnits = 2048 * kGearRatio / inchesPerRotation;
    public Elevator(){
        master = new TalonFX(Constants.Elevator.masterMotorID);
        follower = new TalonFX(Constants.Elevator.followerMotorID);
        master.configFactoryDefault();
        follower.configFactoryDefault();

        master.configForwardSoftLimitThreshold(Constants.Elevator.forwardLimitInches * inchesToNativeUnits);
        master.configForwardSoftLimitEnable(true);

        master.configReverseSoftLimitThreshold(Constants.Elevator.reverseLimitInches * inchesToNativeUnits);
        master.configReverseSoftLimitEnable(true);
        
        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);
        master.setInverted(false);
        follower.setInverted(true);
        
        master.config_kP(0, Constants.Elevator.kP);
        master.config_kI(0, Constants.Elevator.kI);
        master.config_kD(0, Constants.Elevator.kD);

        master.configPeakOutputForward(Constants.Elevator.peakOutForward);
        master.configPeakOutputReverse(Constants.Elevator.peakOutReverse);

        master.setSelectedSensorPosition(0);
        follower.follow(master);
        
    }
    public CommandBase disableLimits() {
      return runOnce(
        () -> {
          master.configForwardSoftLimitEnable(false);
          master.configReverseSoftLimitEnable(false);
        }
        ).withName("Disable Elevator Limits");
  }

    public double getPosition(){
      return master.getSelectedSensorPosition() / inchesToNativeUnits;
    }
    public void goToBottom(){
      setPosition(Constants.Elevator.bottomPos);
    }
    public void goToIntake(){
      setPosition(Constants.Elevator.intakePos);
    }
    public void goToIntakeHigh(){
      setPosition(Constants.Elevator.intakeHighPos);
    }
    public void goToScoreHigh(){
      setPosition(Constants.Elevator.scoreHighPos);
    }
    public void goToScoreMid(){
      setPosition(Constants.Elevator.scoreMidPos);
    }
    public void goToStow(){
      setPosition(Constants.Elevator.stowPos);
    }
    public void setPosition(double inches){
        
        master.set(ControlMode.Position, inches * inchesToNativeUnits, DemandType.ArbitraryFeedForward, 0.05);
    }

    public void setPercentOutput(double v){
        master.set(ControlMode.PercentOutput, v);
    }

    public CommandBase runTestMode(DoubleSupplier d) {
        return runEnd(
          () -> {
            master.set(ControlMode.PercentOutput, d.getAsDouble());
          },
          () -> {
            master.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Elevator");
    }

    public CommandBase testSetpoint() {
        return runEnd(
          () -> {
            goToIntake();
          },
          () -> {
            master.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Elevator Setpoints");
    }

    public CommandBase testBottomSetpoint() {
      return runEnd(
        () -> {
          goToBottom();
        },
        () -> {
          master.set(ControlMode.PercentOutput, 0);
        }
        ).withName("Test Elevator Setpoints");
  }
  
    public double distanceToSetpoint(double setpoint){
        return master.getSelectedSensorPosition() / inchesToNativeUnits - setpoint;
    }

    public double getElevatorVelocity(){
      return master.getSelectedSensorVelocity() / inchesToNativeUnits * 10;
    }

    public double getElevatorPosition(){
      return master.getSelectedSensorPosition() / inchesToNativeUnits;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ElevatorPos", master.getSelectedSensorPosition() / inchesToNativeUnits);
      SmartDashboard.putNumber("Elevator Output", master.getMotorOutputPercent());
    }
}

package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

//WristSubsystem: 
//Position Control via REV Encoder (limits?)
//Controls Y axis of end affecter (up/down)
//Gear ratio:
//ID:
//
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{

    private TalonFX mWrist;  

    private final double kGearRatio = 81;
    private final double kRadiansToNativeUnits = 2048 / 2 / Math.PI * kGearRatio;
    public Wrist(){
        mWrist = new TalonFX(Constants.Wrist.motorID);
        mWrist.configFactoryDefault();

        mWrist.setNeutralMode(NeutralMode.Brake);

        mWrist.configReverseSoftLimitThreshold(Constants.Wrist.reverseLimit * kRadiansToNativeUnits);
        mWrist.configReverseSoftLimitEnable(true);

        mWrist.configForwardSoftLimitThreshold(Constants.Wrist.forwardLimit * kRadiansToNativeUnits);
        mWrist.configForwardSoftLimitEnable(true);

        mWrist.setInverted(false);

        mWrist.config_kP(0, Constants.Wrist.kP);
        mWrist.config_kI(0, Constants.Wrist.kI);
        mWrist.config_kD(0, Constants.Wrist.kD);
        mWrist.setSelectedSensorPosition(0);
    }
    
    public void setPosition(double pos){
        mWrist.set(ControlMode.Position, pos * kRadiansToNativeUnits);
    }

    public void setPercentOutput(double v){
        mWrist.set(ControlMode.PercentOutput, v);
    }
    
    public void goToIntake(){
      setPosition(Constants.Wrist.intakeBackPos);
    }

    public void goToScoreHigh(){
      setPosition(Constants.Wrist.scoreHighPos);
    }
    public void goToHighIntake(){
      setPosition(Constants.Wrist.intakeUpPos);
    }
    public void goToCubeIntake(){
      setPosition(Constants.Wrist.intakeCubePos);
    }
    public void goToScoreMid(){
      setPosition(Constants.Wrist.scoreMidPos);
    }
    public void goToLoad(){
      setPosition(Constants.Wrist.loadPos);
    }

    public void goToCarry(){
      setPosition(Constants.Wrist.carryPos);
    }

    public CommandBase runTestMode(DoubleSupplier d) {
        return runEnd(
          () -> {
            setPercentOutput(d.getAsDouble());
          },
          () -> {
            mWrist.set(ControlMode.PercentOutput, 0.0);
          }
          ).withName("Test Wrist");
    }

    public CommandBase testCarry() {
        return runEnd(
          () -> {
            goToCarry();
          },
          () -> {
            mWrist.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Carry Setpoint");
    }

    public CommandBase testIntake() {
      return runEnd(
        () -> {
          goToIntake();
        },
        () -> {
          mWrist.set(ControlMode.PercentOutput, 0);
        }
        ).withName("Test Intake Setpoint");
  }

  public CommandBase testScore() {
    return runEnd(
      () -> {
        goToScoreHigh();
      },
      () -> {
        mWrist.set(ControlMode.PercentOutput, 0);
      }
      ).withName("Test Score Setpoint");
    }

    public CommandBase zero() {
      return runEnd(
        () -> {
          setPosition(0);
        },
        () -> {
          mWrist.set(ControlMode.PercentOutput, 0);
        }
        ).withName("Zero Wrist");
      }

    public double distanceToSetpoint(double setpoint){
        return mWrist.getSelectedSensorPosition() / kRadiansToNativeUnits - setpoint;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Position", mWrist.getSelectedSensorPosition() / kRadiansToNativeUnits);
    }
}

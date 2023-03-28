// This subsystem is to pivot the arm, and has constraints for how far the arm can turn. 

package frc.robot.subsystems.MiniSystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MoreMath;

public class Pivot extends SubsystemBase{

    private final double kGearRatio = 49;
    private TalonFX pivot = new TalonFX(Constants.Pivot.motorID);
    private final double kRadianstoNativeUnits = 2048 / Math.PI / 2 * kGearRatio;
    private final DutyCycleEncoder mEncoder = new DutyCycleEncoder(0);
    private final ProfiledPIDController mController = new ProfiledPIDController(Constants.Pivot.kP, Constants.Pivot.kI, Constants.Pivot.kD, new TrapezoidProfile.Constraints(1.5, 1));
    
    public Pivot() {
        pivot.configFactoryDefault();
        pivot.setNeutralMode(NeutralMode.Brake);
        pivot.setInverted(true);
        pivot.configOpenloopRamp(0.2);

        mEncoder.setDistancePerRotation(Math.PI * 2);

        File pivotZero = new File("/home/lvuser/constants/PivotZero.txt");
        if (pivotZero.exists()) {
            try {
                Scanner sc = new Scanner(pivotZero);
                Constants.Pivot.zero = Double.parseDouble(sc.nextLine());
                sc.close();
                System.out.println(Constants.Pivot.zero);
            } catch (FileNotFoundException e) {
                System.out.println("Pivot Zero file not found");
            }
        }

        mEncoder.setPositionOffset(Constants.Pivot.zero);
        mController.reset(new State(getEncoderPos(), 0));
        mController.setTolerance(0.05);
    }

    public double getEncoderPos(){
        return -(mEncoder.getDistance());
    }

    /* public void zeroPivot() {
        Constants.Pivot.zero = (mEncoder.getDistance() / (Math.PI*2)) - Constants.Pivot.zero;
        mEncoder.setPositionOffset(Constants.Pivot.zero); */

        public void zeroPivot(boolean run) {
            if (run) {
                mEncoder.setPositionOffset(0);
                Constants.Pivot.zero = mEncoder.getDistance() / (Math.PI*2);
                if(Constants.Pivot.zero < 0){
                    Constants.Pivot.zero += 1;
                }
                //mEncoder.setPositionOffset(Constants.Pivot.zero);
                //NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Pivot").getEntry("Pivot: zero").setDouble(Constants.Pivot.zero);
                
                File pivotZero = new File("/home/lvuser/constants/PivotZero.txt");
                pivotZero.setExecutable(true);
                pivotZero.setReadable(true);
                pivotZero.setWritable(true);
                System.out.println("/home/lvuser/constants/PivotZero.txt");
                try {
                    pivotZero.createNewFile();
                    FileWriter writer = new FileWriter("/home/lvuser/constants/PivotZero.txt");
                    writer.write(Constants.Pivot.zero + "");
                    writer.close();
                } catch (IOException e) {
                    System.out.println("File could not be found when writing to pivot zero");
                }
            }
        }
    
        
    
    public void Run(double voltage) {
        if(getEncoderPos() > Constants.Pivot.forwardLimit && voltage > 0){
            pivot.set(ControlMode.PercentOutput, 0);
        }
        else if(getEncoderPos() < Constants.Pivot.reverseLimit && voltage < 0){
            pivot.set(ControlMode.PercentOutput, 0);
        }
        else{
            pivot.set(ControlMode.PercentOutput, voltage);
        }
    } 

    public void setpos(double angle) {
        pivot.configOpenloopRamp(0.05);
        //System.out.println(MathUtil.clamp(mController.calculate(getEncoderPos(), angle), -0.3, 0.3) + Math.sin(getEncoderPos()) * -0.07);
        Run(MathUtil.clamp(
            mController.calculate(getEncoderPos(),
             new TrapezoidProfile.State(angle, 0),
              new TrapezoidProfile.Constraints(12, 8)) + (Math.sin(getEncoderPos()) * -0.07),
               -1, 1));
    }

    public void setposSlower(double angle) {
        //System.out.println(MathUtil.clamp(mController.calculate(getEncoderPos(), angle), -0.3, 0.3) + Math.sin(getEncoderPos()) * -0.07);
        pivot.configOpenloopRamp(1);
        Run(MathUtil.clamp(
            mController.calculate(getEncoderPos(),
             new TrapezoidProfile.State(angle, 0),
              new TrapezoidProfile.Constraints(12, 8)) + (Math.sin(getEncoderPos()) * -0.07),
               -1, 1));
    }

    public void goToScoreHigh(){
        setpos(Constants.Pivot.scoreHighPos);
    }
    public void goToScoreMid(){
        setpos(Constants.Pivot.scoreMidPos);
    }
    public void goToScoreLow(){
        setpos(Constants.Pivot.scoreLowPos);
    }
    public void goToIntakeHigh(){
        setpos(Constants.Pivot.intakeHighPos);
    }
    public void goToIntake(){
        setpos(Constants.Pivot.intakeBackPos);
    }
    public void goToLoad(){
        setpos(Constants.Pivot.loadPos);
    }
    public void goToShootHigh(){
        setpos(Constants.Pivot.shootHighPos);
    }
    public void goToCloseCubeIntake(){
        setpos(Constants.Pivot.intakeCloseCubePos);
    }
    public void goToIntakeCube(){
        setpos(Constants.Pivot.cubePos);
    }
    public void goToStow(){
        setpos(Constants.Pivot.stowPos);
    }
    public void goToLoadDouble(){
        setposSlower(Constants.Pivot.loadDoublePos);
    }
    public void goToShootMid(){
        setpos(Constants.Pivot.shootMidPos);
    }
    /*
    public CommandBase zeroPivot() {
        return runOnce(
            () -> {
                
            }
            ).withName("Zero Pivot");
    } */

    public CommandBase runTestMode(DoubleSupplier d) {
        return runEnd(
            () -> {
                Run(d.getAsDouble());
              }, 
            () -> {
                pivot.set(ControlMode.PercentOutput, 0.0);
              }
            ).withName("Test Pivot");
    }

    public CommandBase goToZeroCommand() {
        return runEnd(
            () -> {
                setpos(0);
              }, 
            () -> {
                pivot.set(ControlMode.PercentOutput, 0.0);
              }
            ).withName("Zero Pivot").until(() -> Math.abs(distanceToSetpoint(0)) < 0.1);
    }

    public double distanceToSetpoint(double setpoint){
        return getEncoderPos() - setpoint;
    }

    @Override
    public void periodic(){
   
        SmartDashboard.putNumber("Pivot Setpoint", mController.getSetpoint().position);
        SmartDashboard.putNumber("Pivot Setpoint Velocity", mController.getSetpoint().velocity);
        SmartDashboard.putNumber("Pivot Position", getEncoderPos());

    }
}



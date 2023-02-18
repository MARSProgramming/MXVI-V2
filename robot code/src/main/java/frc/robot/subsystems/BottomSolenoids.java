package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomSolenoids extends SubsystemBase {
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 1, 5);

    public BottomSolenoids() {
    }
    public void extend(){
      mSolenoid.set(Value.kForward);
    }
    public void retract(){  
      mSolenoid.set(Value.kReverse);
    }

    public CommandBase toggleBottomSolenoid() {
      return runOnce(
        () -> {
          if (mSolenoid.get()==Value.kOff)
            mSolenoid.set(Value.kReverse); 
          else
            mSolenoid.toggle();
        }).withName("Test Bottom Pneumatics");
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("BottomSolenoidState", mSolenoid.get().toString());
    }

    public void initializeSolenoid(){
      mSolenoid.set(Value.kReverse);
  }
}

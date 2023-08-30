package frc.robot.util;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class CustomXboxController extends XboxController{
    
    public CustomXboxController(int port){
        super(port);
    }
	
    public JoystickButton getAButtonObject(){
        return new JoystickButton(this, XboxController.Button.kA.value);
    }
    public JoystickButton getBButtonObject(){
        return new JoystickButton(this, XboxController.Button.kB.value);
    }
    public JoystickButton getXButtonObject(){
        return new JoystickButton(this, XboxController.Button.kX.value);
    }
    public JoystickButton getYButtonObject(){
        return new JoystickButton(this, XboxController.Button.kY.value);
    }
    public JoystickButton getBackButtonObject(){
        return new JoystickButton(this, XboxController.Button.kBack.value);
    }
    public JoystickButton getStartButtonObject(){
        return new JoystickButton(this, XboxController.Button.kStart.value);
    }
    public JoystickButton getRightStickObject(){
        return new JoystickButton(this, XboxController.Button.kRightStick.value);
    }
    public JoystickButton getLeftStickObject(){
        return new JoystickButton(this, XboxController.Button.kLeftStick.value);
    }
    public JoystickButton getLeftBumperObject(){
        return new JoystickButton(this, XboxController.Button.kLeftBumper.value);
    }
    public JoystickButton getRightBumperObject(){
        return new JoystickButton(this, XboxController.Button.kRightBumper.value);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getLeftTriggerObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger( () -> super.getLeftTriggerAxis() >= Constants.Controller.kTriggerThreshold);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getRightTriggerObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger( () -> super.getRightTriggerAxis() >= Constants.Controller.kTriggerThreshold);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getLeftXJoystickObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getLeftX() >= Constants.Controller.kTriggerThreshold);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getLeftYJoystickObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getLeftY() >= Constants.Controller.kTriggerThreshold);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getRightXJoystickObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getRightX() >= Constants.Controller.kTriggerThreshold);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getRightYJoystickObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getRightY() >= Constants.Controller.kTriggerThreshold);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getLeftDPadObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getPOV() == 270);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getRightDPadObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getPOV() == 90);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getUpDPadObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getPOV() == 0);
    }
    public edu.wpi.first.wpilibj2.command.button.Trigger getDownDPadObject(){
        return new edu.wpi.first.wpilibj2.command.button.Trigger(() -> super.getPOV() == 180);
    }
}

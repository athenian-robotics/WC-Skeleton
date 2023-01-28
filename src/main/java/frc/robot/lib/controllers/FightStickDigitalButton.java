package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class FightStickDigitalButton extends JoystickButton implements FightStickButton {

  private final FightStickInput.input button;

  public FightStickDigitalButton(Joystick stick, int buttonNumber, FightStickInput.input button) {
    super(stick, buttonNumber);
    this.button = button;
  }

  public boolean get(){
    return true;
  }

  @Override
  public FightStickInput.input getButtonInputType() {
    return this.button;
  }
}

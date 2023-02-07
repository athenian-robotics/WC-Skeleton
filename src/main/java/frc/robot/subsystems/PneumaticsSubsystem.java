package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.PneumaticConstants.*;

public class PneumaticsSubsystem extends SubsystemBase {
  private final DoubleSolenoid rightPneumatic;
  private final DoubleSolenoid leftPneumatic;

  public PneumaticsSubsystem() {
    this.rightPneumatic =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortRightA, pneumaticPortRightB);
    this.leftPneumatic =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortLeftA, pneumaticPortLeftB);
  }

  public void extendBothPneumatics() { // Extends pneumatics
    rightPneumatic.set(DoubleSolenoid.Value.kForward);
    leftPneumatic.set(DoubleSolenoid.Value.kForward);
  }

  public void retractBothPneumatics() { // Retracts pneumatics
    rightPneumatic.set(DoubleSolenoid.Value.kReverse);
    leftPneumatic.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendRightPneumatic() { // Extends pneumatic
    rightPneumatic.set(DoubleSolenoid.Value.kForward);
  }

  public void extendLeftPneumatic() { // Extends pneumatic
    leftPneumatic.set(DoubleSolenoid.Value.kForward);
  }

  public void retractRightPneumatic() { // Retracts pneumatic
    rightPneumatic.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractLeftPneumatic() { // Retracts pneumatic
    leftPneumatic.set(DoubleSolenoid.Value.kReverse);
  }
}


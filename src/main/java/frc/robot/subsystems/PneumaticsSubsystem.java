package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.PneumaticConstants.*;

public class PneumaticsSubsystem extends SubsystemBase {
  private final DoubleSolenoid rightIntakePneumatic;
  private final DoubleSolenoid leftIntakePneumatic;

  public PneumaticsSubsystem() {
    this.rightIntakePneumatic =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortRightA, pneumaticPortRightB);
    this.leftIntakePneumatic =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortLeftA, pneumaticPortLeftB);
  }

  public void extendBothPneumatics() { // Extends pneumatics
    rightIntakePneumatic.set(DoubleSolenoid.Value.kForward);
    leftIntakePneumatic.set(DoubleSolenoid.Value.kForward);
  }

  public void retractBothPneumatics() { // Retracts pneumatics
    rightIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
    leftIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendRightPneumatic() { // Extends pneumatic
    rightIntakePneumatic.set(DoubleSolenoid.Value.kForward);
  }

  public void extendLeftPneumatic() { // Extends pneumatic
    leftIntakePneumatic.set(DoubleSolenoid.Value.kForward);
  }

  public void retractRightPneumatic() { // Retracts pneumatic
    rightIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractLeftPneumatic() { // Retracts pneumatic
    leftIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
  }
}


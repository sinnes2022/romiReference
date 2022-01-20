// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoArm extends SubsystemBase {
  private Servo m_servoA;
  /** Creates a new ServoArm. */
  public ServoArm(int pPort) {
    m_servoA = new Servo(pPort);
  }
  
  public void moveTo(double position) {
    m_servoA.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Multi2c {
  public final int I2CAddr;
  private final I2C m_i2c;

  public Multi2c(int addr) {
    I2CAddr = addr;
    m_i2c = new I2C(Port.kOnboard, addr);
  }

  public Multi2c() {
    this(0x70);
  }

  public int getEnabledBuses() {
    byte[] res = new byte[1];
    m_i2c.readOnly(res, 1);
    return res[0];
  }

  public void setEnabledBuses(int... buses) {
    int writeVal = 0;
    for (var b : buses) {
      if (b >= availableBuses() || b < 0) {
        DriverStation.reportError("Invalid i2c bus: " + b, true);
      }
      else {
        writeVal |= 1 << b;
      }
      m_i2c.write(I2CAddr, writeVal);
    }
  }

  public int availableBuses() {
    return 8;
  }
}

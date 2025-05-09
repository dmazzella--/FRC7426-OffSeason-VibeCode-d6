// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Properties;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.shared.Logger;
import frc.robot.shared.PropertiesHttpService;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    try {
      PropertiesHttpService.startService();
    } catch (IOException e) {
      Logger.error(e);
    }
    RobotBase.startRobot(Robot::new);
  }
}

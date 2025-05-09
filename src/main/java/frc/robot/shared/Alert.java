// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.shared;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shared.Elastic.Notification;
import frc.robot.shared.Elastic.Notification.NotificationLevel;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

/** Class for managing persistent alerts to be sent over NetworkTables. */
public class Alert {
  private static Map<String, SendableAlerts> groups = new HashMap<String, SendableAlerts>();

  private Notification alertNotification;
  private NotificationLevel alertLevel;
  private String alertMessage;
  private String alertTitle;

//   private final AlertType type;
  private boolean active = false;
  private boolean notificationDisplayed = false;
  private double activeStartTime = 0.0;

  /**
   * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated,
   * the appropriate entries will be added to NetworkTables.
   *
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  public Alert(String alertTitle, String alertMessage, NotificationLevel type) {
    this("Alerts", alertTitle, alertMessage, type);
  }

  public String getMessage(){
    return alertMessage; 
  }
  /**
   * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
   * entries will be added to NetworkTables.
   *
   * @param group Group identifier, also used as NetworkTables title
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  public Alert(String group, String alertTitle, String alertMessage, NotificationLevel type) {
    if (!groups.containsKey(group)) {
      groups.put(group, new SendableAlerts());
      SmartDashboard.putData(group, groups.get(group));
    }

    this.alertTitle = alertTitle;
    this.alertMessage = alertMessage;
    this.alertLevel = type;

    alertNotification = new Notification();
    alertNotification.setTitle(alertTitle);
    alertNotification.setDescription(alertMessage);
    alertNotification.setLevel(type);
    alertNotification.setDisplayTimeSeconds(5.0);
    alertNotification.setHeight(0.5);

    groups.get(group).alerts.add(this);

  }

  /**
   * Sets whether the alert should currently be displayed. When activated, the alert text will also
   * be sent to the console.
   */
  public void set(boolean active) {
    if (active && !this.active) {
      activeStartTime = Timer.getFPGATimestamp();
      switch (alertLevel) {
        case ERROR:
          DriverStation.reportError(alertMessage, false);
          break;
        case WARNING:
          DriverStation.reportWarning(alertMessage, false);
          break;
        case INFO:
          Logger.println(alertMessage);
          break;
      }
    }
    this.active = active;
    if (!notificationDisplayed) {
      notificationDisplayed = true;
      Elastic.sendNotification(alertNotification);
    }
  }

  /** Updates current alert text. */
  public void setText(String text) {
    if (active && !text.equals(this.alertMessage)) {
      switch (alertLevel) {
        case ERROR:
          DriverStation.reportError(text, false);
          break;
        case WARNING:
          DriverStation.reportWarning(text, false);
          break;
        case INFO:
          Logger.println(text);
          break;
      }
    }
    this.alertMessage = text;
  }

  private static class SendableAlerts implements Sendable {
    public final List<Alert> alerts = new ArrayList<>();

    public String[] getStrings(NotificationLevel type) {
      Predicate<Alert> activeFilter = (Alert x) -> x.alertLevel == type && x.active;
      Comparator<Alert> timeSorter =
          (Alert a1, Alert a2) -> (int) (a2.activeStartTime - a1.activeStartTime);
      return alerts.stream()
          .filter(activeFilter)
          .sorted(timeSorter)
          .map((Alert a) -> a.alertMessage)
          .toArray(String[]::new);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Alerts");
      builder.addStringArrayProperty("errors", () -> getStrings(NotificationLevel.ERROR), null);
      builder.addStringArrayProperty("warnings", () -> getStrings(NotificationLevel.WARNING), null);
      builder.addStringArrayProperty("infos", () -> getStrings(NotificationLevel.INFO), null);
    }
  }

  /** Represents an alert's level of urgency. */
//   public static enum AlertType {
//     /**
//      * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
//      * for problems which will seriously affect the robot's functionality and thus require immediate
//      * attention.
//      */
//     ERROR,

//     /**
//      * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
//      * type for problems which could affect the robot's functionality but do not necessarily require
//      * immediate attention.
//      */
//     WARNING,

//     /**
//      * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
//      * for problems which are unlikely to affect the robot's functionality, or any other alerts
//      * which do not fall under "ERROR" or "WARNING".
//      */
//     INFO
//   }
}
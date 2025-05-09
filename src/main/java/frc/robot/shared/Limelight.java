package frc.robot.shared;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ReconfigurableConfig;

public class Limelight implements ReconfigurableConfig{

    //Setting up Limelight
    protected static Limelight m_instance = new Limelight();

    public static Limelight getInstance() {
    if(m_instance == null) m_instance = new Limelight();
        return m_instance;
    }

    // Our five limelights stored for convenience
    public static final String[] staticLimelights = {"limelight-frleft", "limelight-ftright", "limelight-elleft", "limelight-elright"};

    public static double MaxSpeed = 1;

    public static double autoStage = 1;

    //IDs
    public static double frontRight_Id;
    public static boolean id_Detected; 

    //Calculated X Speeds
    public static double[] txValues = {0.0, 0.0, 0.0, 0.0};

    //Calculated Area
    public static double[] taValues = {0.0, 0.0, 0.0, 0.0};

    //Calculated rotation 
    public static double[] rzValues = {0.0, 0.0, 0.0, 0.0};

    // // Blue Right Side April Tag IDs
    // public static final double[] blueRightIDs = {22, 12, 17};

    // 0 - Teleop Pipeline
    // 1 - Heat Pipeline
    // 2 - Stage 1 Pipeline (Check for Right and Left IDs)
    // 3 - Stage 2 Pipeline (Check for Right and Left IDs)
    // 4 - Stage 3 Pipeline (Check for ID above and Place on Opposite Side Camera)

    //Base PID Values
    public static double kPX = 0.04;
    public static double kPR = 0.05;
    public static double kFPY = 0.2;
    public static double kFDY = 0.01;
    public static double kEPY = 2.5;
    public static double kEDY = 0.25;

    //PID Controllers 
    private static PIDController m_YControllerFrontLeftCamera = new PIDController(kFPY, 0, kFDY);
    private static PIDController m_YControllerFrontRightCamera = new PIDController(kFPY, 0, kFDY);
    private static PIDController m_YControllerElevatorLeftCamera = new PIDController(kEPY, 0, kEDY);
    private static PIDController m_YControllerElevatorRightCamera = new PIDController(kEPY, 0, kEDY);
    private static PIDController[] pidControllers = {m_YControllerFrontLeftCamera, m_YControllerFrontRightCamera, m_YControllerElevatorLeftCamera, m_YControllerElevatorRightCamera};

    // Calculated Y Values
    public static double[] YValues = {0.0, 0.0, 0.0, 0.0};

    /**
     * Returns whether the limelight index is valid
     * @param Limelight Index (Refer to {@link Limelight#staticLimelights})
     * @return Valid Limelight Result
    */
    public static boolean validateLimelight(int limelight_Number) {
        if (limelight_Number >= 0 && limelight_Number <= staticLimelights.length) {
            if (staticLimelights[limelight_Number] != null) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns whether the limelight sees a tag
     * @param Limelight Index (Refer to {@link Limelight#staticLimelights})
     * @return Tag Result
    */
    public static boolean validateTag(int limelight_Number) {
        if (validateLimelight(limelight_Number)) {
            if (LimelightHelpers.getTV(staticLimelights[limelight_Number])) {
                // try {
                //     Logger.println(staticLimelights[limelight_Number] + " sees a tag named ID " + getID(limelight_Number) + "!");
                // } catch (Exception e) {
                //     Logger.println(staticLimelights[limelight_Number] + " got an exception!");
                // }
                return true;
            }
        }
        return false;
    }

    /**
     * Returns the TX of the specified Limelight
     * @param Limelight Index (Refer to {@link Limelight#staticLimelights})
     * @return TX > 0 if reading successful
    */
    public static double getTx(int limelight_Number) {
        if (validateTag(limelight_Number)) {
            return LimelightHelpers.getTX(staticLimelights[limelight_Number]);
        }
        return 0.0;
    }

    /**
     * Returns the TA of the specified Limelight
     * @param Limelight Index (Refer to {@link Limelight#staticLimelights})
     * @return TA > 0 if reading successful
    */
    public static double getTa(int limelight_Number) {
        if (validateTag(limelight_Number)) {
            return LimelightHelpers.getTA(staticLimelights[limelight_Number]);
        }
        return 0.0;
    }

    /**
     * Returns the April Tag ID of the specified Limelight
     * @param Limelight Index (Refer to {@link Limelight#staticLimelights})
     * @return ID if reading successful
    */
    public static double getID(int limelight_Number) {
        if (validateTag(limelight_Number)) {
            return LimelightHelpers.getFiducialID(staticLimelights[limelight_Number]);
        }
        return frontRight_Id;
    }

    /**
     * Sets the pipeline of a specified Limelight
     * @param Limelight Index (Refer to {@link Limelight#staticLimelights})
     * @param Limelight Pipeline Index
    */
    public static void setPipeline(int limelight_Number, int index) {
        if (validateLimelight(limelight_Number)) {
            LimelightHelpers.setPipelineIndex(staticLimelights[limelight_Number], index);
        }
    }
    
    public static void periodic() {
        for (int i = 0; i < staticLimelights.length; i++) {
            // Periodically update TX, TA, and Rotation Values while calculating Y speeds
            txValues[i] = getTx(i);
            taValues[i] = getTa(i);
            YValues[i] = pidControllers[i].calculate(taValues[i]);
            try {
                // Periodically get the robot's orientation based on the tag
                rzValues[i] = LimelightHelpers.getTargetPose_RobotSpace(staticLimelights[i])[4];
            } catch (ArrayIndexOutOfBoundsException e) {
                rzValues[i] = 0.0;
                DriverStation.reportWarning(staticLimelights[i] + " has an invalid rotation value! Check if the Limelight is disconnected!", e.getStackTrace());
            }
        }

        if(getID(1) > 0){
            frontRight_Id = getID(1);
            id_Detected = true; 
        } else {
            id_Detected = false; 
        }
      
        if(Constants.Dashboard.kSendDebug) {
            SmartDashboard.putNumber("Front Left Limeight TX", txValues[0]);
            SmartDashboard.putNumber("Front Right Limeight TX", txValues[1]);
            SmartDashboard.putNumber("Elevator Left Limeight TX", txValues[2]);
            SmartDashboard.putNumber("Elevator Right Limeight TX", txValues[3]);

            SmartDashboard.putNumber("Front Left RZ", rzValues[0]);
            SmartDashboard.putNumber("Front Right RZ", rzValues[1]);
            SmartDashboard.putNumber("Elevator Left RZ", rzValues[2]);
            SmartDashboard.putNumber("Elevator Right RZ", rzValues[3]);

            SmartDashboard.putNumber("Front Left Limeight Ta", taValues[0]);
            SmartDashboard.putNumber("Front Right Limeight Ta", taValues[1]);
            SmartDashboard.putNumber("Elevator Left Limeight Ta", taValues[2]);
            SmartDashboard.putNumber("Elevator Right Limeight Ta", taValues[3]);

            SmartDashboard.putNumber("Front Right ID Value", frontRight_Id);
            SmartDashboard.putBoolean("Front Right ID Detected", id_Detected);
        }

    }

    /**
     * Calculates the required rate of rotation needed to center onto a valid tag
     * @return Angular rate to rotate at, in degrees per second.
     */
    public double CenterRotationRate(int limelight_Number, double targetRZ, double threshold) {
        double rotaionRate;
        if (validateTag(limelight_Number)) {
            if (threshold < Math.abs(targetRZ - rzValues[limelight_Number])) {
                rotaionRate = (targetRZ - rzValues[limelight_Number]) * kPR;
            } else {
                rotaionRate = 0;
                // Logger.println(staticLimelights[limelight_Number] + " has finished centering on rotation rate!");
            }
            // Logger.println("Rotation (1): " + rotaionRate);
            return rotaionRate;
        }
        // If the camera's Angle is within a certain limit, don't do anything. If not, rotate to get in that limit
        return 0.0;
    }

    /**
     * Calculates the required X speed needed to center on a valid tag 
     * @return X speed to move at, in meters per second.
     */
    public double CenterTx(int limelight_Number, double targetTX, double threshold) {
        double xSpeed;
        if (validateTag(limelight_Number)) {
            if (threshold < Math.abs(targetTX - txValues[limelight_Number])) {
                xSpeed = (targetTX - txValues[limelight_Number]) * kPX * 0.75;
            } else {                  
                xSpeed = 0;
                // Logger.println(staticLimelights[limelight_Number] + " has finished centering on x-axis!");
            }
            return xSpeed;
        }
        // If the camera's TX is within a certain limit, don't do anything. If not, drive to get in that limit
        return 0.0;
    }

    /**
     * Calculates the required Y speed needed for distance on a valid tag
     * @return Y speed to move at, in meters per second.
     */
    public double CenterTy(int limelight_Number, double threshold, double targetArea) {
        double ySpeed;
        if (validateTag(limelight_Number)) {
            pidControllers[limelight_Number].setSetpoint(targetArea);
            double error = (targetArea - taValues[limelight_Number]);
            if (threshold < Math.abs(error)) {
                // ySpeed = error * kPY;
                ySpeed = YValues[limelight_Number] * 0.6;
            } else {                   
                ySpeed = 0;
                // Logger.println(staticLimelights[limelight_Number] + " has finished centering on distance!");
            }
            return ySpeed;
        }
        // If the front left camera's TY is within a certain limit, don't do anything. If not, drive to get in that limit
        return 0.0;
    }

    public boolean isCloseLeft(){
        if (taValues[1] > 1.5) {
            return true;
        }
        return false;
    }

    public boolean isCloseRight(){
        if (taValues[0] > 1.5) {
            return true;
        }
        return false;
    }

    @Override
    public void reconfigure() {
        for (int i = 0; i < pidControllers.length; i++) {
            if (pidControllers[i].toString().contains("Elevator")) {
                pidControllers[i] = new PIDController(kEPY, 0, kEDY);
            } else {
                pidControllers[i] = new PIDController(kFPY, 0, kFDY);
            }
        }
    }
}

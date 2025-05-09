package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ConstantsTest {
    @Test
    void visionConstants_AprilTagApproachDistance_ShouldBeTenCentimeters() {
        // Assert
        assertEquals(0.10, Constants.VisionConstants.APRILTAG_APPROACH_DISTANCE_METERS, 0.001);
    }
    
    @Test
    void joystickConstants_ButtonCount_ShouldBeThirtyTwo() {
        // Assert
        assertEquals(32, Constants.JoystickConstants.BUTTON_COUNT);
    }
    
    @Test
    void joystickConstants_AprilTagButtons_ShouldContainButtonsOneToEight() {
        // Assert
        int[] expectedButtons = {1, 2, 3, 4, 5, 6, 7, 8};
        assertArrayEquals(expectedButtons, Constants.JoystickConstants.APRILTAG_BUTTONS);
    }
    
    @Test
    void joystickPort_ShouldBeOne() {
        // Assert
        assertEquals(1, Constants.JOYSTICK_PORT);
    }
}
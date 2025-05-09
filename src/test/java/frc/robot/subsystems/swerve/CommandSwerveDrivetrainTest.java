package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

class CommandSwerveDrivetrainTest {

    // private CommandSwerveDrivetrain drivetrain;
    // private SwerveDrivetrainConstants drivetrainConstants;
    // private SwerveModuleConstants<?, ?, ?>[] modules;

    // @BeforeEach
    // public void setUp() {
    //     drivetrainConstants = mock(SwerveDrivetrainConstants.class);
    //     modules = new SwerveModuleConstants<?, ?, ?>[4];
    //     for (int i = 0; i < 4; i++) {
    //         modules[i] = mock(SwerveModuleConstants.class);
    //     }
    //     drivetrain = new CommandSwerveDrivetrain(drivetrainConstants, modules);
    // }

    // @Test
    // void testApplyRequest() {
    //     SwerveRequest request = mock(SwerveRequest.class);
    //     Command command = drivetrain.applyRequest(() -> request);
    //     assertNotNull(command);
    // }

    // @Test
    // void testSysIdQuasistatic() {
    //     Command command = drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    //     assertNotNull(command);
    // }

    // @Test
    // void testSysIdDynamic() {
    //     Command command = drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward);
    //     assertNotNull(command);
    // }

    // @Test
    // void testPeriodic() {
    //     try (MockedStatic<DriverStation> mockedDriverStation = mockStatic(DriverStation.class)) {
    //         mockedDriverStation.when(DriverStation::isDisabled).thenReturn(true);
    //         mockedDriverStation.when(DriverStation::getAlliance).thenReturn(java.util.Optional.of(Alliance.Blue));
    //         drivetrain.periodic();
    //         // Add assertions or verifications as needed
    //         assertTrue(DriverStation.isDisabled());
    //         assertEquals(Alliance.Blue, DriverStation.getAlliance().orElse(null));
    //     }
    // }

    // @Test
    // void testGetDriveMotorWatts() {
    //     assertThrows(IndexOutOfBoundsException.class, () -> drivetrain.getDriveMotorWatts(-1));
    //     assertThrows(IndexOutOfBoundsException.class, () -> drivetrain.getDriveMotorWatts(4));
    // }

    // @Test
    // void testGetSteerMotorWatts() {
    //     assertThrows(IndexOutOfBoundsException.class, () -> drivetrain.getSteerMotorWatts(-1));
    //     assertThrows(IndexOutOfBoundsException.class, () -> drivetrain.getSteerMotorWatts(4));
    // }
}

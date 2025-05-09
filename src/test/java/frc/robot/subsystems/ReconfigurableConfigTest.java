package frc.robot.subsystems;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import java.io.File;
import java.net.URL;
import java.util.Enumeration;
import java.util.List;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

class ReconfigurableConfigTest {

    // private ClassLoader classLoader;
    // private Enumeration<URL> resources;

    // @SuppressWarnings("unchecked")
    // @BeforeEach
    // void setUp() {
    //     classLoader = mock(ClassLoader.class);
    //     resources = mock(Enumeration.class);
    // }

    // @Test
    // void testFindReconfigurableConfigs() throws Exception {
    //     // Mocking the resources
    //     URL resource = new File("src/test/resources/frc/robot/subsystems").toURI().toURL();
    //     when(resources.hasMoreElements()).thenReturn(true, false);
    //     when(resources.nextElement()).thenReturn(resource);
    //     when(classLoader.getResources(anyString())).thenReturn(resources);

    //     // Setting the class loader
    //     Thread.currentThread().setContextClassLoader(classLoader);

    //     // Calling the method
    //     List<Class<?>> result = ReconfigurableConfig.findReconfigurableConfigs();

    //     // Verifying the result
    //     assertEquals(1, result.size());
    //     assertEquals(ReconfigurableConfig.class, result.get(0));
    // }

    // @Test
    // void testFindReconfigurableConfigsNoResources() throws Exception {
    //     // Mocking the resources
    //     when(resources.hasMoreElements()).thenReturn(false);
    //     when(classLoader.getResources(anyString())).thenReturn(resources);

    //     // Setting the class loader
    //     Thread.currentThread().setContextClassLoader(classLoader);

    //     // Calling the method
    //     List<Class<?>> result = ReconfigurableConfig.findReconfigurableConfigs();

    //     // Verifying the result
    //     assertEquals(0, result.size());
    // }
}
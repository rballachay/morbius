package ca.mcgill.ecse458.tcpserver.unit;

import ca.mcgill.ecse458.tcpserver.driver.RobotIdService;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import static org.junit.jupiter.api.Assertions.assertEquals;

@SpringBootTest
public class RobotIdServiceTest {

    @Autowired
    private RobotIdService robotIdService;
    @AfterEach
    public void resetIdCounter() {
        robotIdService.resetIdCounter();
    }

    @Test
    public void testGetAndIncrement() {
        assertEquals(1, robotIdService.get());
        robotIdService.increment();
        assertEquals(2, robotIdService.getAndIncrement());
        assertEquals(3, robotIdService.get());
    }

    @Test
    public void testReset() {
        assertEquals(1, robotIdService.get());
        robotIdService.increment();
        robotIdService.increment();
        robotIdService.increment();
        assertEquals(4, robotIdService.get());

        robotIdService.resetIdCounter();
        assertEquals(1, robotIdService.get());
    }
}

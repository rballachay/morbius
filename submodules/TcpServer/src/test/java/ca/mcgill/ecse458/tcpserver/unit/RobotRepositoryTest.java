package ca.mcgill.ecse458.tcpserver.unit;

import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import java.util.HashSet;

import static org.junit.jupiter.api.Assertions.*;

@SpringBootTest
public class RobotRepositoryTest {

    @Autowired
    private RobotRepository robotRepository;

    private final RobotThread rt1 = new RobotThread(null, 1),
                                rt2 = new RobotThread(null, 2),
                                rt3 = new RobotThread(null, 3);

    @AfterEach
    public void clearRepository() {
        robotRepository.deleteRobotById(1);
        robotRepository.deleteRobotById(2);
        robotRepository.deleteRobotById(3);
    }

    public void addAllRobotsToRepository() {
        robotRepository.addRobotById(1, rt1);
        robotRepository.addRobotById(2, rt2);
        robotRepository.addRobotById(3, rt3);
    }

    @Test
    public void testAddRobots() {
        boolean result;

        result = robotRepository.addRobotById(1, rt1);
        assertTrue(result);
        result = robotRepository.addRobotById(2, rt2);
        assertTrue(result);
        result = robotRepository.addRobotById(3, rt3);
        assertTrue(result);

        result = robotRepository.addRobotById(1, rt1);
        assertFalse(result);

        assertEquals(3, robotRepository.getRepositorySize());
    }

    @Test
    public void testGetRobots() {
        addAllRobotsToRepository();

        assertEquals(rt2, robotRepository.getRobotById(2));
        assertNull(robotRepository.getRobotById(4));
    }

    @Test
    public void testUpdateRobotId() {
        boolean result;
        addAllRobotsToRepository();

        result = robotRepository.updateRobotId(1, 4);
        assertTrue(result);
        assertEquals(4, robotRepository.getRobotById(4).getRobotId());

        result = robotRepository.updateRobotId(1, 5);
        assertFalse(result);
        result = robotRepository.updateRobotId(2, 3);
        assertFalse(result);
    }

    @Test
    public void testDeleteRobots() {
        boolean result;
        addAllRobotsToRepository();

        result = robotRepository.deleteRobotById(2);
        assertTrue(result);

        result = robotRepository.deleteRobotById(4);
        assertFalse(result);

        assertEquals(2, robotRepository.getRepositorySize());
    }

    @Test
    public void testGetRobotList() {
        addAllRobotsToRepository();

        HashSet<RobotThread>  robots = new HashSet<>();
        robots.add(rt1);
        robots.add(rt2);
        robots.add(rt3);

        HashSet<RobotThread> result = new HashSet<>(robotRepository.getRobots());

        assertEquals(robots, result);
    }
}

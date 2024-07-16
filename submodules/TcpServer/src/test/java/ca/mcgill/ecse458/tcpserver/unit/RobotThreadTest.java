package ca.mcgill.ecse458.tcpserver.unit;

import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import ca.mcgill.ecse458.tcpserver.robot.RobotType;
import org.junit.jupiter.api.Test;
import org.springframework.boot.test.context.SpringBootTest;

import java.util.concurrent.locks.Condition;

import static org.junit.jupiter.api.Assertions.*;

@SpringBootTest
public class RobotThreadTest {

    private final Command command = Command.FORWARD;
    private final Status status = Status.SUCCESS;
    private final int distance = 10;
    private final double angle = 20.0;
    private final double result = 30.0;
    private final String text = "spongebobsquarepants12345";

    @Test
    public void testMessages() {
        RobotThread rt = new RobotThread(null, 1);
        rt.putMessage(new Message.MessageBuilder(1, command, status).
                intData(new int[]{distance}).floatData(new double[]{angle}).result(result).text(text).build());
        assertEquals(1, rt.getQueueSize());

        Message m = rt.removeMessage();
        assertEquals(1, m.getId());
        assertEquals(command, m.getCommand());
        assertEquals(status, m.getStatus());
        assertArrayEquals(new int[]{distance}, m.getIntData());
        assertArrayEquals(new double[]{angle}, m.getFloatData());
        assertEquals(result, m.getResult());

        assertEquals(0, rt.getQueueSize());
    }

    @Test
    public void testRobotType() {
        RobotThread rt = new RobotThread(null, 1);
        rt.setRobotType(RobotType.HOVERBOT);
        assertEquals(RobotType.HOVERBOT, rt.getRobotType());

        rt.setRobotType(RobotType.BOEBOT);
        assertEquals(RobotType.BOEBOT, rt.getRobotType());
    }

    @Test
    public void testRobotId() {
        RobotThread rt = new RobotThread(null, 1);
        assertEquals(1, rt.getRobotId());

        rt.setRobotId(2);
        assertEquals(2, rt.getRobotId());
    }

    @Test
    public void testRobotConditionGetter() {
        RobotThread rt = new RobotThread(null, 1);
        Condition hasResponse = rt.getHasResponse();
        assertNotNull(hasResponse);
    }
}

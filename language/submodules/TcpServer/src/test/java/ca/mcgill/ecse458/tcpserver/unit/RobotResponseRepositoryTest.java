package ca.mcgill.ecse458.tcpserver.unit;

import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import static org.junit.jupiter.api.Assertions.assertEquals;

@SpringBootTest
public class RobotResponseRepositoryTest {

    @Autowired
    RobotResponseRepository robotResponseRepository;
    private final Message message = new Message.MessageBuilder(1, Command.FORWARD, Status.DISPATCHED).
            intData(new int[]{10}).build();

    @AfterEach
    public void clearRepository() {
        robotResponseRepository.emptyQueue();
    }

    @Test
    public void testPutMessage() {
        assertEquals(0, robotResponseRepository.getQueueSize());
        robotResponseRepository.putMessage(message);
        assertEquals(1, robotResponseRepository.getQueueSize());
    }

    @Test
    public void testRemoveMessage() {
        assertEquals(0, robotResponseRepository.getQueueSize());
        robotResponseRepository.putMessage(message);
        robotResponseRepository.putMessage(message);
        assertEquals(2, robotResponseRepository.getQueueSize());
        Message m = robotResponseRepository.removeMessage();
        assertEquals(message.getId(), m.getId());
        assertEquals(message.getCommand(), m.getCommand());
        assertEquals(message.getStatus(), m.getStatus());
        assertEquals(message.getIntData(), m.getIntData());
        assertEquals(message.getFloatData(), m.getFloatData());
        assertEquals(1, robotResponseRepository.getQueueSize());
    }

    @Test
    public void testEmptyQueue() {
        assertEquals(0, robotResponseRepository.getQueueSize());
        robotResponseRepository.putMessage(message);
        robotResponseRepository.putMessage(message);
        robotResponseRepository.putMessage(message);
        assertEquals(3, robotResponseRepository.getQueueSize());
        robotResponseRepository.emptyQueue();
        assertEquals(0, robotResponseRepository.getQueueSize());
    }

    @Test
    public void testGetMessageById() {
        assertEquals(0, robotResponseRepository.getQueueSize());
        robotResponseRepository.putMessage(message);
        robotResponseRepository.putMessage(message);
        assertEquals(2, robotResponseRepository.getQueueSize());
        Message m = robotResponseRepository.getResponseById(1);
        assertEquals(message.getId(), m.getId());
        assertEquals(message.getCommand(), m.getCommand());
        assertEquals(message.getStatus(), m.getStatus());
        assertEquals(message.getIntData(), m.getIntData());
        assertEquals(message.getFloatData(), m.getFloatData());
        assertEquals(1, robotResponseRepository.getQueueSize());
    }
}

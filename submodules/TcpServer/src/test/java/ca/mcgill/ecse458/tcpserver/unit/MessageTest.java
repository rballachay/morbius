package ca.mcgill.ecse458.tcpserver.unit;

import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.message.dto.RobotDto;
import ca.mcgill.ecse458.tcpserver.robot.RobotType;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.springframework.boot.test.context.SpringBootTest;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

@SpringBootTest
public class MessageTest {

    private final int id = 1;
    private final Command command = Command.FORWARD;
    private final Status dispatchedStatus = Status.DISPATCHED;
    private final int distance = 10;
    private final double angle = 20.0;
    private final double result = 30.0;
    private final String text = "spongebobsquarepants12345";
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final String jsonResponse =
            //"{\"id\":1,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"distance\":10.0,\"angle\":20.0,\"result\":30.0}";
            "{\"id\":1,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[],\"result\":30.0,\"text\":\"spongebobsquarepants12345\"}";

    @Test
    public void testBuildMessageWithAllFields() {
        Message m = new Message.MessageBuilder(id, command, dispatchedStatus).
                intData(new int[]{distance}).floatData(new double[]{angle}).result(result).text(text).build();
        assertEquals(id, m.getId());
        assertEquals(command, m.getCommand());
        assertEquals(dispatchedStatus, m.getStatus());
        assertArrayEquals(new int[]{distance}, m.getIntData());
        assertArrayEquals(new double[]{angle}, m.getFloatData());
        assertEquals(result, m.getResult());
        assertEquals(text, m.getText());
    }

    @Test
    public void testBuildMessageWithMissingFields() {
        Message m = new Message.MessageBuilder(id, command, dispatchedStatus).intData(new int[]{distance}).build();
        assertEquals(id, m.getId());
        assertEquals(command, m.getCommand());
        assertEquals(dispatchedStatus, m.getStatus());
        assertArrayEquals(new int[]{distance}, m.getIntData());
        assertArrayEquals(new double[] {}, m.getFloatData());
        assertEquals(0.0, m.getResult());
        assertEquals("", m.getText());
    }

    @Test
    public void testCreateMessageWithJsonCreator() {
        try {
            Message response = objectMapper.readValue(jsonResponse, Message.class);
            assertEquals(id, response.getId());
            assertEquals(command, response.getCommand());
            assertEquals(dispatchedStatus, response.getStatus());
            assertArrayEquals(new int[]{distance}, response.getIntData());
            assertArrayEquals(new double[] {}, response.getFloatData());
            assertEquals(result, response.getResult());
            assertEquals(text, response.getText());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void testEqualsOverride() {
        Message m1 = new Message.MessageBuilder(id, command, dispatchedStatus).
                intData(new int[]{distance}).floatData(new double[]{angle}).result(result).text(text).build();
        Message m2 = new Message.MessageBuilder(id, command, dispatchedStatus).
                intData(new int[]{distance}).floatData(new double[]{angle}).result(result).text(text).build();
        assertEquals(m1, m2);

        Message m3 = new Message.MessageBuilder(id, command, Status.SUCCESS).
                intData(new int[]{distance}).floatData(new double[]{angle}).result(result).text(text).build();
        assertNotEquals(m1, m3);

        Message m4 = new Message.MessageBuilder(id, command, dispatchedStatus).
                intData(new int[]{distance}).floatData(new double[]{angle}).build();
        assertNotEquals(m1, m4);

        assertNotEquals(null, m1);
        assertNotEquals(new RobotDto(1, RobotType.HOVERBOT), m1);
    }
}

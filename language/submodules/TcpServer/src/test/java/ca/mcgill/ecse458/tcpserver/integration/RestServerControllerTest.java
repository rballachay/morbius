package ca.mcgill.ecse458.tcpserver.integration;

import ca.mcgill.ecse458.tcpserver.driver.RobotIdService;
import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.message.dto.GetRobotsDto;
import ca.mcgill.ecse458.tcpserver.message.dto.MessageRequestDto;
import ca.mcgill.ecse458.tcpserver.message.dto.RobotDto;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import ca.mcgill.ecse458.tcpserver.robot.RobotType;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.client.TestRestTemplate;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;

import java.util.ArrayList;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
public class RestServerControllerTest {

    @Autowired
    private TestRestTemplate client;
    @Autowired
    private RobotRepository robotRepository;
    @Autowired
    private RobotIdService robotIdService;
    private final RobotThread rt1 = new RobotThread(null, 1),
                              rt2 = new RobotThread(null, 2),
                              rt3 = new RobotThread(null, 3);
    private final double param = 123.456;
    private final Status status = Status.DISPATCHED;

    @BeforeEach
    public void setupRobots() {
        rt1.setRobotType(RobotType.HOVERBOT);
        rt2.setRobotType(RobotType.BOEBOT);
        rt3.setRobotType(RobotType.BOEBOT);
        robotRepository.addRobotById(1, rt1);
        robotRepository.addRobotById(2, rt2);
        robotRepository.addRobotById(3, rt3);
    }

    @AfterEach
    public void clearRepository() {
        for (int i = 1; i < 10; i++) {
            robotRepository.deleteRobotById(i);
        }
        robotIdService.resetIdCounter();
    }

    @Test
    public void testGetRobots() {
        ResponseEntity<GetRobotsDto> response = client.getForEntity("/robots", GetRobotsDto.class);

        assertNotNull(response);
        ArrayList<RobotDto> results = (ArrayList<RobotDto>) Objects.requireNonNull(response.getBody()).getRobots();

        assertEquals(3, results.size());
        assertTrue(results.contains(new RobotDto(rt1)));
        assertTrue(results.contains(new RobotDto(rt2)));
        assertTrue(results.contains(new RobotDto(rt3)));
    }

    @Test
    public void testForwardCommand() {
        ResponseEntity<Message> response = client.postForEntity(
                "/forward", new MessageRequestDto(1, param), Message.class);

        Message command = new Message.MessageBuilder(1, Command.FORWARD, status).floatData(new double[]{param}).
                build();
        assertNotNull(response);
        assertEquals(HttpStatus.OK, response.getStatusCode());
        assertEquals(command, response.getBody());

        assertEquals(1, rt1.getQueueSize());
        assertEquals(command, rt1.removeMessage());
    }

    @Test
    public void testBackwardCommand() {
        ResponseEntity<Message> response = client.postForEntity(
                "/backward", new MessageRequestDto(1, param), Message.class);

        Message command = new Message.MessageBuilder(1, Command.BACKWARD, status).floatData(new double[]{param}).
                build();
        assertNotNull(response);
        assertEquals(HttpStatus.OK, response.getStatusCode());
        assertEquals(command, response.getBody());

        assertEquals(1, rt1.getQueueSize());
        assertEquals(command, rt1.removeMessage());
    }

    @Test
    public void testTurnRightCommand() {
        ResponseEntity<Message> response = client.postForEntity(
                "/turnright", new MessageRequestDto(1, param), Message.class);

        Message command = new Message.MessageBuilder(1, Command.TURNRIGHT, status).floatData(new double[]{param}).
                build();
        assertNotNull(response);
        assertEquals(HttpStatus.OK, response.getStatusCode());
        assertEquals(command, response.getBody());

        assertEquals(1, rt1.getQueueSize());
        assertEquals(command, rt1.removeMessage());
    }

    @Test
    public void testTurnLeftCommand() {
        ResponseEntity<Message> response = client.postForEntity(
                "/turnleft", new MessageRequestDto(1, param), Message.class);

        Message command = new Message.MessageBuilder(1, Command.TURNLEFT, status).floatData(new double[]{param}).
                build();
        assertNotNull(response);
        assertEquals(HttpStatus.OK, response.getStatusCode());
        assertEquals(command, response.getBody());

        assertEquals(1, rt1.getQueueSize());
        assertEquals(command, rt1.removeMessage());
    }

    @Test
    public void testPingCommand() {
        ResponseEntity<Message> response = client.postForEntity(
                "/ping", new MessageRequestDto(1, param), Message.class);

        Message command = new Message.MessageBuilder(1, Command.PING, status).build();
        assertNotNull(response);
        assertEquals(HttpStatus.OK, response.getStatusCode());
        assertEquals(command, response.getBody());

        assertEquals(1, rt1.getQueueSize());
        assertEquals(command, rt1.removeMessage());
    }

    @Test
    public void testChangeId() {
        ResponseEntity<Message> response = client.postForEntity(
                "/changeid", new MessageRequestDto(1, 4), Message.class);

        Message command = new Message.MessageBuilder(1, Command.CHANGEID, Status.SUCCESS).build();
        assertNotNull(response);
        assertEquals(HttpStatus.OK, response.getStatusCode());
        assertEquals(command, response.getBody());

        assertEquals(3, robotRepository.getRepositorySize());
        assertNull(robotRepository.getRobotById(1));

        RobotThread changed = robotRepository.getRobotById(4);
        assertNotNull(changed);
        assertEquals(rt1, changed);

        ResponseEntity<String> badResponse = client.postForEntity(
                "/changeid", new MessageRequestDto(2, 3), String.class);

        assertNotNull(badResponse);
        assertEquals(HttpStatus.BAD_REQUEST, badResponse.getStatusCode());
        assertEquals("The repository already contains a robot with the given new ID", badResponse.getBody());
    }

    @Test
    public void testInvalidIdCommands() {
        ResponseEntity<String> badResponse;

        badResponse = client.postForEntity("/forward", new MessageRequestDto(10, param), String.class);

        assertNotNull(badResponse);
        assertEquals(HttpStatus.BAD_REQUEST, badResponse.getStatusCode());
        assertEquals("No robot with the given ID exists", badResponse.getBody());

        assertEquals(0, rt1.getQueueSize());

        badResponse = client.postForEntity("/backward", new MessageRequestDto(10, param), String.class);

        assertNotNull(badResponse);
        assertEquals(HttpStatus.BAD_REQUEST, badResponse.getStatusCode());
        assertEquals("No robot with the given ID exists", badResponse.getBody());

        assertEquals(0, rt1.getQueueSize());

        badResponse = client.postForEntity("/turnright", new MessageRequestDto(10, param), String.class);

        assertNotNull(badResponse);
        assertEquals(HttpStatus.BAD_REQUEST, badResponse.getStatusCode());
        assertEquals("No robot with the given ID exists", badResponse.getBody());

        assertEquals(0, rt1.getQueueSize());

        badResponse = client.postForEntity("/turnleft", new MessageRequestDto(10, param), String.class);

        assertNotNull(badResponse);
        assertEquals(HttpStatus.BAD_REQUEST, badResponse.getStatusCode());
        assertEquals("No robot with the given ID exists", badResponse.getBody());

        assertEquals(0, rt1.getQueueSize());

        badResponse = client.postForEntity("/ping", new MessageRequestDto(10, param), String.class);

        assertNotNull(badResponse);
        assertEquals(HttpStatus.BAD_REQUEST, badResponse.getStatusCode());
        assertEquals("No robot with the given ID exists", badResponse.getBody());

        assertEquals(0, rt1.getQueueSize());

        badResponse = client.postForEntity("/changeid", new MessageRequestDto(10, param), String.class);

        assertNotNull(badResponse);
        assertEquals(HttpStatus.BAD_REQUEST, badResponse.getStatusCode());
        assertEquals("No robot with the given ID exists", badResponse.getBody());
    }

}

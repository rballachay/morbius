package ca.mcgill.ecse458.tcpserver.integration;

import ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint;
import ca.mcgill.ecse458.tcpserver.driver.RobotIdService;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import ca.mcgill.ecse458.tcpserver.robot.RobotType;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.context.ApplicationContext;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;

import static org.junit.jupiter.api.Assertions.assertEquals;

@Disabled("test is passing but conflicts with AiResponseWriterTest and AiTcpEndpointTest when run at the same time")
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.MethodName.class)
@SpringBootTest
public class AiCommandReaderTest {

    @Autowired
    private RobotRepository robotRepository;
    @Autowired
    private RobotIdService robotIdService;
    @Autowired
    private ApplicationContext applicationContext;
    private Socket aiSocket;
    private AiTcpEndpoint ate;
    private PrintWriter out;
    private BufferedReader in;
    private final RobotThread rt1 = new RobotThread(null, 1),
                              rt2 = new RobotThread(null, 2),
                              rt3 = new RobotThread(null, 3);
    private final ObjectMapper objectMapper = new ObjectMapper();

    private final String[] outgoingCommands =
            {"{\"id\":1,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":1,\"command\":\"BACKWARD\",\"status\":\"DISPATCHED\",\"intData\":[],\"floatData\":[20.0],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":1,\"command\":\"TURNLEFT\",\"status\":\"DISPATCHED\",\"intData\":[30],\"floatData\":[40.0],\"result\":0.0,\"text\":\"\"}",};

    @BeforeAll
    public void startATE() throws IOException {
        ate = new AiTcpEndpoint();
        applicationContext.getAutowireCapableBeanFactory().autowireBean(ate);
        ate.start();

        aiSocket = new Socket("localhost", 50049);
        out = new PrintWriter(aiSocket.getOutputStream(), true);
        in = new BufferedReader(new InputStreamReader(aiSocket.getInputStream()));
    }

    @BeforeEach
    public void setupRobot() {
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

    @AfterAll
    public void stopATE() throws IOException {
        out.close();
        aiSocket.close();

        ate.getAiCommandReader().stop();
        ate.getAiResponseWriter().stop();
        ate.stop();
    }

    @Test
    public void testSendOneMessageToRobotThread() {
        try {
            out.println(outgoingCommands[0]);

            Thread.sleep(100);

            assertEquals(1, rt1.getQueueSize());
            Message command = rt1.removeMessage();
            Message original = objectMapper.readValue(outgoingCommands[0], Message.class);
            assertEquals(original, command);
        } catch (InterruptedException | JsonProcessingException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testSendThreeMessagesToRobotThread() {
        try {
            for (String s: outgoingCommands) {
                out.println(s);
            }

            Thread.sleep(100);

            assertEquals(3, rt1.getQueueSize());

            for (String s: outgoingCommands) {
                Message command = rt1.removeMessage();
                Message original = objectMapper.readValue(s, Message.class);
                assertEquals(original, command);
            }
        } catch (InterruptedException | JsonProcessingException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testGetRobots() {
        try {
            //out.println("{\"id\":0,\"command\":\"GETROBOTS\",\"status\":\"DISPATCHED\",\"distance\":0.0,\"angle\":0.0,\"result\":0.0}");
            out.println("{\"id\":0,\"command\":\"GETROBOTS\",\"status\":\"DISPATCHED\",\"intData\":[],\"floatData\":[],\"result\":0.0,\"text\":\"\"}");

            String response = in.readLine();
            assertEquals("{\"robots\":[{\"id\":1,\"robotType\":\"HOVERBOT\"},{\"id\":2,\"robotType\":\"BOEBOT\"},{\"id\":3,\"robotType\":\"BOEBOT\"}]}",
                    response);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testSendCommandWithInvalidId() {
        try {
            //out.println("{\"id\":10,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"distance\":10.0,\"angle\":0.0,\"result\":0.0}");
            out.println("{\"id\":10,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[],\"result\":0.0,\"text\":\"\"}");

            String response = in.readLine();
            //assertEquals("{\"id\":10,\"command\":\"FORWARD\",\"status\":\"INVALID\",\"distance\":0.0,\"angle\":0.0,\"result\":0.0}", response);
            assertEquals("{\"id\":10,\"command\":\"FORWARD\",\"status\":\"INVALID\",\"intData\":[10],\"floatData\":[],\"result\":0.0,\"text\":\"\"}", response);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

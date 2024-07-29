package ca.mcgill.ecse458.tcpserver.integration;

import ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint;
import ca.mcgill.ecse458.tcpserver.driver.RobotIdService;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository;
import org.junit.jupiter.api.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.context.ApplicationContext;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.MethodName.class)
@SpringBootTest
public class AiTcpEndpointTest {

    @Autowired
    private RobotRepository robotRepository;
    @Autowired
    private RobotIdService robotIdService;
    @Autowired
    private RobotResponseRepository robotResponseRepository;
    @Autowired
    private ApplicationContext applicationContext;
    private Socket aiSocket;
    private AiTcpEndpoint ate;
    private PrintWriter aiOut;
    private BufferedReader aiIn;
    private final int[] indexOrder = {2, 0, 1};
    private final String[] outgoingCommands =
            {"{\"id\":1,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":1,\"command\":\"BACKWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":1,\"command\":\"TURNLEFT\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":2,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":2,\"command\":\"BACKWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":2,\"command\":\"TURNLEFT\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":3,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":3,\"command\":\"BACKWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}",
             "{\"id\":3,\"command\":\"TURNLEFT\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[12.3],\"result\":0.0,\"text\":\"\"}"};

    private final String[] incomingResponses =
            {"{\"id\":1,\"command\":\"FORWARD\",\"status\":\"SUCCESS\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":1,\"command\":\"BACKWARD\",\"status\":\"SUCCESS\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":1,\"command\":\"TURNLEFT\",\"status\":\"FAILURE\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":2,\"command\":\"FORWARD\",\"status\":\"SUCCESS\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":2,\"command\":\"BACKWARD\",\"status\":\"SUCCESS\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":2,\"command\":\"TURNLEFT\",\"status\":\"FAILURE\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":3,\"command\":\"FORWARD\",\"status\":\"SUCCESS\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":3,\"command\":\"BACKWARD\",\"status\":\"SUCCESS\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",
             "{\"id\":3,\"command\":\"TURNLEFT\",\"status\":\"FAILURE\",\"intData\":[10],\"floatData\":[12.3],\"result\":12.3,\"text\":\"\"}",};

    @BeforeAll
    public void startATE() throws IOException {
        ate = new AiTcpEndpoint();
        applicationContext.getAutowireCapableBeanFactory().autowireBean(ate);
        ate.start();

        aiSocket = new Socket("localhost", 50049);
        aiOut = new PrintWriter(aiSocket.getOutputStream(), true);
        aiIn = new BufferedReader(new InputStreamReader(aiSocket.getInputStream()));
    }

    @BeforeEach
    public void emptyQueue() {
        robotResponseRepository.emptyQueue();
    }

    @AfterEach
    public void resetRobots() {
        for (int i = 1; i < 10; i++) {
            robotRepository.deleteRobotById(i);
        }
        robotIdService.resetIdCounter();
        robotResponseRepository.emptyQueue();
    }

    @AfterAll
    public void stopATE() throws IOException {
        aiOut.close();
        aiIn.close();
        aiSocket.close();

        ate.getAiCommandReader().stop();
        ate.getAiResponseWriter().stop();
        ate.stop();
    }

    @Test
    public void testClassGetters() {
        assertNotNull(ate.getAiCommandReader());
        assertNotNull(ate.getAiResponseWriter());
    }

    @Test
    public void testSendOneMessageToOneRobotThread() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter robotOut = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader robotIn = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            robotOut.println("HOVERBOT");
            robotIn.readLine();
            aiOut.println(outgoingCommands[0]);

            Thread.sleep(100);

            String serverCommand = robotIn.readLine();
            assertEquals(outgoingCommands[0], serverCommand);
            robotOut.println(incomingResponses[0]);

            Thread.sleep(100);

            String serverResponse = aiIn.readLine();
            assertEquals(incomingResponses[0], serverResponse);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testSendThreeMessagesToOneRobotThreadAndRespondInOrder() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter robotOut = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader robotIn = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            robotOut.println("HOVERBOT");
            robotIn.readLine();

            for (int i = 0; i < 3; i++) {
                aiOut.println(outgoingCommands[i]);
            }

            Thread.sleep(100);

            for (int i = 0; i < 3; i++) {
                String serverCommand = robotIn.readLine();
                assertEquals(outgoingCommands[i], serverCommand);
                robotOut.println(incomingResponses[i]);
            }

            Thread.sleep(100);

            for (int i = 0; i < 3; i++) {
                String serverResponse = aiIn.readLine();
                assertEquals(incomingResponses[i], serverResponse);
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testSendThreeMessagesToOneRobotThreadAndRespondOutOfOrder() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter robotOut = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader robotIn = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            robotOut.println("HOVERBOT");
            robotIn.readLine();

            for (int i = 0; i < 3; i++) {
                aiOut.println(outgoingCommands[i]);
            }

            Thread.sleep(100);

            for (int i = 0; i < 3; i++) {
                String serverCommand = robotIn.readLine();
                assertEquals(outgoingCommands[i], serverCommand);
                robotOut.println(incomingResponses[indexOrder[i]]);
            }

            Thread.sleep(100);

            for (int i: indexOrder) {
                String serverResponse = aiIn.readLine();
                assertEquals(incomingResponses[i], serverResponse);
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testSendOneMessageToThreeRobotThreadsAndRespond() {
        try (
                Socket robotSocket1 = new Socket("localhost", 50050);
                PrintWriter robotOut1 = new PrintWriter(robotSocket1.getOutputStream(), true);
                BufferedReader robotIn1 = new BufferedReader(new InputStreamReader(robotSocket1.getInputStream()));
                Socket robotSocket2 = new Socket("localhost", 50050);
                PrintWriter robotOut2 = new PrintWriter(robotSocket2.getOutputStream(), true);
                BufferedReader robotIn2 = new BufferedReader(new InputStreamReader(robotSocket2.getInputStream()));
                Socket robotSocket3 = new Socket("localhost", 50050);
                PrintWriter robotOut3 = new PrintWriter(robotSocket3.getOutputStream(), true);
                BufferedReader robotIn3 = new BufferedReader(new InputStreamReader(robotSocket3.getInputStream()));
        ) {
            robotOut1.println("HOVERBOT");
            robotIn1.readLine();
            robotOut2.println("HOVERBOT");
            robotIn2.readLine();
            robotOut3.println("HOVERBOT");
            robotIn3.readLine();

            ArrayList<String> validResponses = new ArrayList<>();

            for (int i = 0; i <= 6; i+=3) {
                aiOut.println(outgoingCommands[i]);
                validResponses.add(incomingResponses[i]);
            }

            Thread.sleep(100);
            String serverCommand;

            serverCommand = robotIn1.readLine();
            assertEquals(outgoingCommands[0], serverCommand);
            robotOut1.println(incomingResponses[0]);

            serverCommand = robotIn2.readLine();
            assertEquals(outgoingCommands[3], serverCommand);
            robotOut2.println(incomingResponses[3]);

            serverCommand = robotIn3.readLine();
            assertEquals(outgoingCommands[6], serverCommand);
            robotOut3.println(incomingResponses[6]);

            Thread.sleep(100);

            for (int i = 0; i <= 6; i+=3) {
                String serverResponse = aiIn.readLine();
                assertTrue(validResponses.contains(serverResponse));
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testSendThreeMessagesToThreeRobotThreadsAndRespond() {
        try (
                Socket robotSocket1 = new Socket("localhost", 50050);
                PrintWriter robotOut1 = new PrintWriter(robotSocket1.getOutputStream(), true);
                BufferedReader robotIn1 = new BufferedReader(new InputStreamReader(robotSocket1.getInputStream()));
                Socket robotSocket2 = new Socket("localhost", 50050);
                PrintWriter robotOut2 = new PrintWriter(robotSocket2.getOutputStream(), true);
                BufferedReader robotIn2 = new BufferedReader(new InputStreamReader(robotSocket2.getInputStream()));
                Socket robotSocket3 = new Socket("localhost", 50050);
                PrintWriter robotOut3 = new PrintWriter(robotSocket3.getOutputStream(), true);
                BufferedReader robotIn3 = new BufferedReader(new InputStreamReader(robotSocket3.getInputStream()));
        ) {
            robotOut1.println("HOVERBOT");
            robotIn1.readLine();
            robotOut2.println("HOVERBOT");
            robotIn2.readLine();
            robotOut3.println("HOVERBOT");
            robotIn3.readLine();

            ArrayList<String> validResponses = new ArrayList<>(List.of(incomingResponses));

            for (int i = 0; i < 9; i++) {
                aiOut.println(outgoingCommands[i]);
            }

            Thread.sleep(100);
            String serverCommand;

            for (int i = 0; i < 3; i++) {
                serverCommand = robotIn1.readLine();
                assertEquals(outgoingCommands[i], serverCommand);
                robotOut1.println(incomingResponses[i]);

                serverCommand = robotIn2.readLine();
                assertEquals(outgoingCommands[i+3], serverCommand);
                robotOut2.println(incomingResponses[i+3]);

                serverCommand = robotIn3.readLine();
                assertEquals(outgoingCommands[i+6], serverCommand);
                robotOut3.println(incomingResponses[i+6]);
            }

            Thread.sleep(100);

            for (int i = 0; i < 9; i++) {
                String serverResponse = aiIn.readLine();
                assertTrue(validResponses.contains(serverResponse));
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testGetRobots() {
        try (
                Socket robotSocket1 = new Socket("localhost", 50050);
                PrintWriter robotOut1 = new PrintWriter(robotSocket1.getOutputStream(), true);
                BufferedReader robotIn1 = new BufferedReader(new InputStreamReader(robotSocket1.getInputStream()));
                Socket robotSocket2 = new Socket("localhost", 50050);
                PrintWriter robotOut2 = new PrintWriter(robotSocket2.getOutputStream(), true);
                BufferedReader robotIn2 = new BufferedReader(new InputStreamReader(robotSocket2.getInputStream()));
                Socket robotSocket3 = new Socket("localhost", 50050);
                PrintWriter robotOut3 = new PrintWriter(robotSocket3.getOutputStream(), true);
                BufferedReader robotIn3 = new BufferedReader(new InputStreamReader(robotSocket3.getInputStream()));
        ) {
            robotOut1.println("HOVERBOT");
            robotIn1.readLine();
            robotOut2.println("BOEBOT");
            robotIn2.readLine();
            robotOut3.println("BOEBOT");
            robotIn3.readLine();

            aiOut.println("{\"id\":0,\"command\":\"GETROBOTS\",\"status\":\"DISPATCHED\",\"intData\":[],\"floatData\":[],\"result\":0.0,\"text\":\"\"}");

            String response = aiIn.readLine();
            assertEquals("{\"robots\":[{\"id\":1,\"robotType\":\"HOVERBOT\"},{\"id\":2,\"robotType\":\"BOEBOT\"},{\"id\":3,\"robotType\":\"BOEBOT\"}]}",
                    response);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testSendCommandWithInvalidId() {
        try {
            aiOut.println("{\"id\":10,\"command\":\"FORWARD\",\"status\":\"DISPATCHED\",\"intData\":[10],\"floatData\":[],\"result\":0.0,\"text\":\"\"}");

            String response = aiIn.readLine();
            assertEquals("{\"id\":10,\"command\":\"FORWARD\",\"status\":\"INVALID\",\"intData\":[10],\"floatData\":[],\"result\":0.0,\"text\":\"\"}", response);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

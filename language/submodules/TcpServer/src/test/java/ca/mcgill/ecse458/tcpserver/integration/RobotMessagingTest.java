package ca.mcgill.ecse458.tcpserver.integration;

import ca.mcgill.ecse458.tcpserver.driver.RobotIdService;
import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.assertEquals;

@SpringBootTest
public class RobotMessagingTest {

    @Autowired
    private RobotRepository robotRepository;
    @Autowired
    private RobotIdService robotIdService;
    @Autowired
    private RobotResponseRepository robotResponseRepository;
    private final int id = 1;
    private final Command moveCommand = Command.FORWARD;
    private final Command turnCommand = Command.TURNRIGHT;
    private final Status successStatus = Status.SUCCESS;
    private final Status failureStatus = Status.FAILURE;
    private final Status dispatchedStatus = Status.DISPATCHED;
    private final int distance = 10;
    private final double angle = 20.0;
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final String jsonTemplate =
            //"{\"id\":1,\"command\":\"%s\",\"status\":\"%s\",\"distance\":%.1f,\"angle\":%.1f,\"result\":%.1f}";
            "{\"id\":1,\"command\":\"%s\",\"status\":\"%s\",\"intData\":%s,\"floatData\":%s,\"result\":%.1f,\"text\":%s}";

    private final Message[] messages = {new Message.MessageBuilder(1, Command.FORWARD, dispatchedStatus).
                    intData(new int[]{distance, 20}).floatData(new double[]{angle, 30.0}).build(),
            new Message.MessageBuilder(1, Command.BACKWARD, dispatchedStatus).intData(new int[]{distance, 20}).
                    floatData(new double[]{angle, 30.0}).build(),
            new Message.MessageBuilder(1, Command.TURNLEFT, dispatchedStatus).intData(new int[]{distance, 20}).
                    floatData(new double[]{angle, 30.0}).build(),
            new Message.MessageBuilder(1, Command.TURNRIGHT, dispatchedStatus).intData(new int[]{distance, 20}).
                    floatData(new double[]{angle, 30.0}).build(),
            new Message.MessageBuilder(1, Command.PING, dispatchedStatus).build()};

    @AfterEach
    public void clearRepository() {
        for (int i = 1; i < 10; i++) {
            robotRepository.deleteRobotById(i);
        }
        robotIdService.resetIdCounter();
        robotResponseRepository.emptyQueue();
    }

    @Test
    public void testSendMoveCommandAndReceiveSuccessResponse() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter out = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            String serverResponse;

            out.println("HOVERBOT");
            in.readLine();

            robotRepository.getRobotById(1).putMessage(
                    new Message.MessageBuilder(id, moveCommand, dispatchedStatus).intData(new int[]{distance}).build());

            serverResponse = in.readLine();
            assertEquals(String.format(jsonTemplate, moveCommand, dispatchedStatus, Arrays.toString(new int[]{distance}), "[]", 0.0, "\"\""), serverResponse);

            String robotJsonResponse = String.format(jsonTemplate, moveCommand, successStatus, Arrays.toString(new int[]{distance}), "[]", (float) distance, "\"\"");
            out.println(robotJsonResponse);

            Thread.sleep(100);

            assertEquals(1, robotResponseRepository.getQueueSize());
            Message robotResponse = robotResponseRepository.removeMessage();
            assertEquals(objectMapper.readValue(robotJsonResponse, Message.class), robotResponse);
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            System.exit(1);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    @Test
    public void testSendTurnCommandAndReceiveSuccessResponse() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter out = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            String serverResponse;

            out.println("HOVERBOT");
            in.readLine();

            robotRepository.getRobotById(1).putMessage(
                    new Message.MessageBuilder(id, turnCommand, dispatchedStatus).floatData(new double[]{angle}).build());

            serverResponse = in.readLine();
            assertEquals(String.format(jsonTemplate, turnCommand, dispatchedStatus, "[]", Arrays.toString(new double[]{angle}), 0.0, "\"\""), serverResponse);

            String robotJsonResponse = String.format(jsonTemplate, turnCommand, successStatus, "[]", Arrays.toString(new double[]{angle}), angle, "\"\"");
            out.println(robotJsonResponse);

            Thread.sleep(100);

            assertEquals(1, robotResponseRepository.getQueueSize());
            Message robotResponse = robotResponseRepository.removeMessage();
            assertEquals(objectMapper.readValue(robotJsonResponse, Message.class), robotResponse);
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            System.exit(1);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    @Test
    public void testSendMoveCommandAndReceiveFailureResponse() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter out = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            String serverResponse;

            out.println("HOVERBOT");
            in.readLine();

            robotRepository.getRobotById(1).putMessage(
                    new Message.MessageBuilder(id, moveCommand, dispatchedStatus).intData(new int[]{distance}).build());

            serverResponse = in.readLine();
            assertEquals(String.format(jsonTemplate, moveCommand, dispatchedStatus, Arrays.toString(new int[]{distance}), "[]", 0.0, "\"\""), serverResponse);

            String robotJsonResponse = String.format(jsonTemplate, moveCommand, failureStatus, Arrays.toString(new int[]{distance}), "[]", (float) distance, "\"\"");
            out.println(robotJsonResponse);

            Thread.sleep(100);

            assertEquals(1, robotResponseRepository.getQueueSize());
            Message robotResponse = robotResponseRepository.removeMessage();
            assertEquals(objectMapper.readValue(robotJsonResponse, Message.class), robotResponse);
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            System.exit(1);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    @Test
    public void testSendTurnCommandAndReceiveFailureResponse() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter out = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            String serverResponse;

            out.println("HOVERBOT");
            in.readLine();

            robotRepository.getRobotById(1).putMessage(
                    new Message.MessageBuilder(id, turnCommand, dispatchedStatus).floatData(new double[]{angle}).build());

            serverResponse = in.readLine();
            assertEquals(String.format(jsonTemplate, turnCommand, dispatchedStatus, "[]", Arrays.toString(new double[]{angle}), 0.0, "\"\""), serverResponse);

            String robotJsonResponse = String.format(jsonTemplate, turnCommand, failureStatus, "[]", Arrays.toString(new double[]{angle}), angle, "\"\"");
            out.println(robotJsonResponse);

            Thread.sleep(100);

            assertEquals(1, robotResponseRepository.getQueueSize());
            Message robotResponse = robotResponseRepository.removeMessage();
            assertEquals(objectMapper.readValue(robotJsonResponse, Message.class), robotResponse);
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            System.exit(1);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    @Test
    public void testSendFiveCommandsAndReceiveFiveResponsesInOrder() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter out = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            out.println("HOVERBOT");
            in.readLine();

            for (Message message : messages) {
                robotRepository.getRobotById(1).putMessage(message);
            }

            for (Message message : messages) {
                out.println(in.readLine());
            }

            Thread.sleep(100);

            assertEquals(5, robotResponseRepository.getQueueSize());

            for (Message message : messages) {
                Message robotResponse = robotResponseRepository.removeMessage();
                assertEquals(message, robotResponse);
            }
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            System.exit(1);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

}

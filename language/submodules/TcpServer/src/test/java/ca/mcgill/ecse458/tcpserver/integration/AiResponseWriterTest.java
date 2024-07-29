package ca.mcgill.ecse458.tcpserver.integration;

import ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint;
import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.context.ApplicationContext;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

import static org.junit.jupiter.api.Assertions.assertEquals;

@Disabled("test is passing but conflicts with AiCommandReaderTest and AiTcpEndpointTest when run at the same time")
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.MethodName.class)
@SpringBootTest
public class AiResponseWriterTest {

    @Autowired
    private RobotResponseRepository robotResponseRepository;
    @Autowired
    private ApplicationContext applicationContext;
    private Socket aiSocket;
    private AiTcpEndpoint ate;
    private BufferedReader in;
    private final ObjectMapper objectMapper = new ObjectMapper();

    private final Message[] messages = {new Message.MessageBuilder(1, Command.FORWARD, Status.SUCCESS).
                    intData(new int[]{10}).result(10.0).build(),
            new Message.MessageBuilder(2, Command.TURNRIGHT, Status.SUCCESS).floatData(new double[]{20.0}).
                    result(20.0).build(),
            new Message.MessageBuilder(3, Command.PING, Status.FAILURE).result(0.0).build()};

    @BeforeAll
    public void startATE() throws IOException {
        ate = new AiTcpEndpoint();
        applicationContext.getAutowireCapableBeanFactory().autowireBean(ate);
        ate.start();

        aiSocket = new Socket("localhost", 50049);
        in = new BufferedReader(new InputStreamReader(aiSocket.getInputStream()));
    }

    @AfterEach
    public void clearResponseRepository() {
        robotResponseRepository.emptyQueue();
    }

    @AfterAll
    public void stopATE() throws IOException {
        in.close();
        aiSocket.close();

        ate.getAiCommandReader().stop();
        ate.getAiResponseWriter().stop();
        ate.stop();
    }

    @Test
    public void testReadOneMessageFromResponseRepository() {
        try {
            robotResponseRepository.putMessage(messages[0]);

            Thread.sleep(100);

            String serverResponse = in.readLine();
            Message response = objectMapper.readValue(serverResponse, Message.class);

            assertEquals(messages[0], response);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void testReadThreeMessagesFromResponseRepository() {
        try {
            for (Message m: messages) {
                robotResponseRepository.putMessage(m);
            }

            Thread.sleep(100);

            for (Message m: messages) {
                String serverResponse = in.readLine();
                Message response = objectMapper.readValue(serverResponse, Message.class);
                assertEquals(m, response);
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

}

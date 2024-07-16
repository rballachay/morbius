package ca.mcgill.ecse458.tcpserver.controller;

import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Autowired;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

/**
 * <p>
 * Threaded class housing the AI response writer.
 * </p>
 * <p>
 * Checks the robot response repository for incoming responses, serializes them from Message objects into JSON and
 * sends them to the AI for further processing.
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint
 * @see ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository
 * @author Ze Yuan Fu
 */
public class AiResponseWriter extends Thread {

    /**
     * Reference to the robot response repository which is used to retrieve robot responses
     * @see ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository
     */
    @Autowired
    private RobotResponseRepository robotResponseRepository;

    /**
     * Printwriter of the AI endpoint socket that the thread sends serialized robot responses to
     */
    private final PrintWriter out;

    /**
     * ObjectMapper used to serialize Messages into JSON strings
     */
    private final ObjectMapper objectMapper;

    /**
     * Constructor for the response writer
     * @param out       PrintWriter to be used for writing to the TCP endpoint
     */
    public AiResponseWriter(PrintWriter out) {
        super("AI Response Writer");
        this.objectMapper = new ObjectMapper();
        this.out = out;
    }

    public void run() {
        System.out.println("starting AI response writer");
        String jsonResponse;

        try {
            while(true) {
                Message response = robotResponseRepository.removeMessage();     // read next response from repository

                System.out.printf("sending response to AI: id: %d; command: %s; status: %s; int params: %s; float params: %s; result: %.1f; text: %s%n",
                        response.getId(), response.getCommand(), response.getStatus(),
                        Arrays.toString(response.getIntData()), Arrays.toString(response.getFloatData()),
                        response.getResult(), response.getText());

                jsonResponse = objectMapper.writeValueAsString(response);       // serialize into JSON
                out.println(jsonResponse);                                      // send to AI through TCP endpoint
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

    }
}

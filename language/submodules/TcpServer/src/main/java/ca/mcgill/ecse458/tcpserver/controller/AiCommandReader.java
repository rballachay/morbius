package ca.mcgill.ecse458.tcpserver.controller;

import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.message.dto.GetRobotsDto;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Autowired;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

import static java.util.Objects.isNull;

/**
 * <p>
 * Threaded class housing the AI command reader.
 * </p>
 *
 * <p>
 * Listens on the TCP endpoint for incoming commands from the AI, deserializes them from JSON into Message objects and
 * sends them to the recipient robot. If the robot with the target ID does not exist, an INVALID response is returned
 * to the AI.
 * </p>
 *
 * <p>
 * If a server-related command (get list of robots, etc) is received, it is executed immediately and
 * the result is returned to the AI using the endpoint.
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint
 * @see ca.mcgill.ecse458.tcpserver.repository.RobotRepository ca.mcgill.ecse458.tcpserver.repository.RobotRepository
 * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
 * @author Ze Yuan Fu
 */
public class AiCommandReader extends Thread {

    /**
     * Reference to the server's robot repository, used to retrieve robot threads to send commands to
     * @see ca.mcgill.ecse458.tcpserver.repository.RobotRepository ca.mcgill.ecse458.tcpserver.repository.RobotRepository
     */
    @Autowired
    private RobotRepository robotRepository;

    /**
     * BufferedReader of the AI endpoint socket that the thread reads to get incoming commands
     */
    private final BufferedReader in;

    /**
     * Printwriter of the AI endpoint socket that the thread sends server-related commands to
     */
    private final PrintWriter out;

    /**
     * ObjectMapper used to deserialize JSON strings into Messages
     */
    private final ObjectMapper objectMapper;

    /**
     * Constructor for the command reader
     * @param in        BufferedReader to be used for reading from the TCP endpoint
     * @param out       PrintWriter to be used for writing to the TCP endpoint
     */
    public AiCommandReader(BufferedReader in, PrintWriter out) {
        super("AI Command Reader");
        this.objectMapper = new ObjectMapper();
        this.in = in;
        this.out = out;
    }

    public void run() {
        System.out.println("starting AI command reader");
        String jsonCommand;

        try {
            while(true) {
                jsonCommand = in.readLine();        // read next JSON input from AI
                Message command = objectMapper.readValue(jsonCommand, Message.class);       // deserialize

                System.out.printf("received command from AI: id: %d; command: %s; int params: %s; float params: %s%n",
                        command.getId(), command.getCommand(), Arrays.toString(command.getIntData()),
                        Arrays.toString(command.getFloatData()));

                switch (command.getCommand()) {     // execute server commands immediately, otherwise hand over to robot
                    case GETROBOTS:     GetRobotsDto result = new GetRobotsDto(robotRepository.getRobotDtos());
                                        out.println(objectMapper.writeValueAsString(result));
                                        break;

                    default:            if (!isNull(robotRepository.getRobotById(command.getId()))) {               // robot exists
                                            robotRepository.getRobotById(command.getId()).putMessage(command);
                                        } else {                                                                    // robot does not exist
                                            Message response = new Message.MessageBuilder(
                                                    command.getId(), command.getCommand(), Status.INVALID).
                                                    intData(command.getIntData()).floatData(command.getFloatData()).build();
                                            out.println(objectMapper.writeValueAsString(response));
                                        }
                                        break;
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

    }
}

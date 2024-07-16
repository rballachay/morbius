package ca.mcgill.ecse458.tcpserver.robot;

import ca.mcgill.ecse458.tcpserver.driver.ServerSocketListener;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Autowired;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.text.MessageFormat;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * <p>
 * Threaded class representing a connection to a robot.
 * </p>
 *
 * <p>
 * Messages are received using a deque, read, serialized to JSON and sent to the robot via the TCP socket.
 * The response, also in JSON format, is read, deserialized into a Message and sent to the RobotResponseRepository for
 * further processing.
 * </p>
 *
 * <p>
 * Messages are sent and expected as JSON strings in the following format:
 * </p>
 *
 * <p>
 * {"id":1,"command":"FORWARD","status":"DISPATCHED","intData":[10],"floatData":[12.3],"result":0.0,"text":""}
 * </p>
 *
 * <p>
 * - id: integer                                                                                                        <br>
 * - command: string whose value corresponds to one of the enumerations in ca.mcgill.ecse458.tcpserver.message.Command  <br>
 * - status: string whose value corresponds to one of the enumerations in ca.mcgill.ecse458.tcpserver.message.Status    <br>
 * - intData: array of integer command parameters                                                                       <br>
 * - floatData: array of float command parameters                                                                       <br>
 * - result: double (no limit on the number of significant figures)                                                     <br>
 * - text: string
 * </p>
 *
 * <p>
 * The contents of the deque are modified using a lock and wait-notify methods for thread-safety.
 * </p>
 *
 * <p>
 * Future improvements: use protocol buffers as the communication medium between the server and robots to enable
 * platform-agnosticism
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
 * @see ca.mcgill.ecse458.tcpserver.robot.RobotType ca.mcgill.ecse458.tcpserver.robot.RobotType
 * @see ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository
 * @author Ze Yuan Fu
 */
public class RobotThread extends Thread {

    /**
     * Server's robot response repository to which the robot's responses will be sent
     * @see ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository ca.mcgill.ecse458.tcpserver.repository.RobotResponseRepository
     */
    @Autowired
    private RobotResponseRepository robotResponseRepository;
    @Autowired
    private ServerSocketListener serverSocketListener;

    /**
     * TCP socket backing the PrintWriter and BufferedReader used to communicate with the robot
     */
    private final Socket socket;

    /**
     * Queue for incoming command messages
     * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
     */
    private final Deque<Message> messageQueue;

    /**
     * ObjectMapper used to serialize and deserialize Messages to/from JSON strings
     */
    private final ObjectMapper objectMapper;

    /**
     * Type of the robot
     * @see ca.mcgill.ecse458.tcpserver.robot.RobotType ca.mcgill.ecse458.tcpserver.robot.RobotType
     */
    private RobotType robotType;

    /**
     * ID of the robot
     */
    private int id;

    /**
     * Lock to maintain thread safety when accessing the Deque
     */
    private final Lock lock;

    /**
     * Condition for when the incoming message queue is not empty
     */
    private final Condition queueNotEmpty;

    /**
     * Condition for when the robot thread has sent a response message to the robot response repository
     */
    private final Condition hasResponse;

    /**
     * Constructor for the robot thread
     * @param socket        TCP socket associated with the robot
     * @param id            ID of the robot
     */
    public RobotThread(Socket socket, int id) {
        this.socket = socket;
        this.id = id;
        this.messageQueue = new ArrayDeque<>();
        this.objectMapper = new ObjectMapper();
        this.lock = new ReentrantLock();
        this.queueNotEmpty = lock.newCondition();
        this.hasResponse = lock.newCondition();
    }

    public void run() {
        try (   // create PrintWriter and BufferedReader to write to and read from the TCP socket
                PrintWriter out = new PrintWriter(socket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        ) {
            // after establishing a connection, the robot first sends its type to the server
            //String rt = in.readLine();
            char[] buf = new char[2048];        // read chars into char array then turn it into a string to be parsed
            int bytes = in.read(buf);
            String rt = new String(buf).trim();
            setRobotType(RobotType.valueOf(rt));
            out.println(MessageFormat.format("set robottype to {0}", rt));
            serverSocketListener.addRobotToRepository(this, this.id);

            while(true) {
                // get first message in the queue, serialize to JSON and send to the robot via the TCP connection
                Message m = removeMessage();
                System.out.printf("sending command: id: %d; command: %s; int params: %s; float params: %s%n",
                        m.getId(), m.getCommand(), Arrays.toString(m.getIntData()), Arrays.toString(m.getFloatData()));

                String jsonMessage = objectMapper.writeValueAsString(m);
                out.println(jsonMessage);

                // read JSON response from robot, deserialize to Message and add to RobotResponseRepository
                //String jsonResponse = in.readLine();
                buf = new char[2048];           // read chars into char array then turn it into a string to be parsed
                Arrays.fill(buf, ' ');
                bytes = in.read(buf);
                String jsonResponse = new String(buf).trim();
                Message r = objectMapper.readValue(jsonResponse, Message.class);
                System.out.printf("received response: id: %d; command: %s; status: %s; int params: %s; float params: %s; result: %.1f; text: %s%n",
                        r.getId(), r.getCommand(), r.getStatus(), Arrays.toString(r.getIntData()),
                        Arrays.toString(r.getFloatData()), r.getResult(), r.getText());

                // add the response to the response repository and signal the condition associated with this thread
                robotResponseRepository.putMessage(r);
                lock.lock();
                try {
                    hasResponse.signal();
                } finally {
                    lock.unlock();
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    /**
     * Adds a Message to the end of this robot's Message queue
     * @param message       Message to be added to the queue
     */
    public void putMessage(Message message) {
        lock.lock();
        try {
            messageQueue.addLast(message);
            queueNotEmpty.signal();
        } finally {
            lock.unlock();
        }
    }

    /**
     * Removes and returns the first message from this robot's Message queue. If the queue is empty, waits until a
     * message is added, then retrieves it.
     * @return              First message in the queue
     */
    public Message removeMessage() {
        lock.lock();
        try {
            while(messageQueue.isEmpty()) {
                queueNotEmpty.await();
            }

            return messageQueue.pop();
        } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        } finally {
            lock.unlock();
        }
    }

    /**
     * Returns the number of elements in this robot's Message queue
     * @return              Number of elements in the queue
     */
    public int getQueueSize() {
        lock.lock();
        try {
            return messageQueue.size();
        } finally {
            lock.unlock();
        }
    }

    /**
     * Returns the type of this robot
     * @return              Type of this robot
     */
    public RobotType getRobotType() {
        return robotType;
    }

    /**
     * Sets the type of this robot to the given type
     * @param rt            RobotType to set
     */
    public void setRobotType(RobotType rt) {
        this.robotType = rt;
    }

    /**
     * Returns the ID of this robot
     * @return              ID of this robot
     */
    public int getRobotId() {
        return id;
    }

    /**
     * Sets the ID of this robot to the given ID
     * @param id            New ID
     */
    public void setRobotId(int id) {
        this.id = id;
    }

    /**
     * Returns the hasResponse condition for use with the RobotResponseRepository
     * @return              hasResponse condition object
     */
    public Condition getHasResponse() {
        return hasResponse;
    }

}

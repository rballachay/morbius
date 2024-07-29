package ca.mcgill.ecse458.tcpserver.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.ApplicationContext;
import org.springframework.stereotype.Component;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * <p>
 * Threaded class housing the TCP endpoint used by the Prometheus Meta AI to stream incoming commands and outgoing robot
 * responses in real time.
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
 * The endpoint maintains references to two threads: a command reader which reads incoming commands from the AI and
 * submits them to the recipient robots for execution, and a response writer which reads robot responses from the
 * robot response repository and streams them back to the AI.
 * </p>
 *
 * <p>
 * Two separate threads are used because calls to both BufferedReader.readLine() (for the command reader) and
 * Condition.await() (for the response writer) are blocking, so attempting to write a response while waiting to read an
 * incoming command within the same thread, and vice-versa, will not work.
 * </p>
 *
 * <p>
 * Future improvements: use protocol buffers as the communication medium between the server and AI to enable
 * platform-agnosticism
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.controller.AiCommandReader ca.mcgill.ecse458.tcpserver.controller.AiCommandReader
 * @see ca.mcgill.ecse458.tcpserver.controller.AiResponseWriter ca.mcgill.ecse458.tcpserver.controller.AiResponseWriter
 * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
 * @author Ze Yuan Fu
 */
@Component
public class AiTcpEndpoint extends Thread {

    /**
     * Spring ApplicationContext used to autowire the subthreads created by this class
     */
    @Autowired
    private ApplicationContext applicationContext;

    /**
     * Reference to the AI command reader subthread
     * @see ca.mcgill.ecse458.tcpserver.controller.AiCommandReader ca.mcgill.ecse458.tcpserver.controller.AiCommandReader
     */
    private AiCommandReader aiCommandReader;

    /**
     * Reference to the AI response writer subthread
     * @see ca.mcgill.ecse458.tcpserver.controller.AiResponseWriter ca.mcgill.ecse458.tcpserver.controller.AiResponseWriter
     */
    private AiResponseWriter aiResponseWriter;

    /**
     * Constructor for the AI TCP endpoint
     */
    public AiTcpEndpoint() {
        super("AI TCP Endpoint");
    }

    public void run() {
        int port = 50049;               // port that the AI will use to connect to the server

        System.out.println("starting AI TCP endpoint");

        try (   // create network resources required for the command reader and response writer
                ServerSocket serverSocket = new ServerSocket(port);
                Socket aiSocket = serverSocket.accept();
                PrintWriter out = new PrintWriter(aiSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(aiSocket.getInputStream()));
        ) {
            System.out.println("AI connection established, starting subthreads");
            aiCommandReader = new AiCommandReader(in, out);
            aiResponseWriter = new AiResponseWriter(out);
            applicationContext.getAutowireCapableBeanFactory().autowireBean(aiCommandReader);
            applicationContext.getAutowireCapableBeanFactory().autowireBean(aiResponseWriter);
            aiCommandReader.start();
            aiResponseWriter.start();
            while(true) {                       // sleep to ensure that the references to the network resources are not
                sleep(Long.MAX_VALUE);          // garbage collected
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    /**
     * Returns a reference to the command reader (mostly for testing purposes)
     * @return      The AI command reader subthread of the endpoint thread
     */
    public AiCommandReader getAiCommandReader() {
        return aiCommandReader;
    }

    /**
     * Returns a reference to the command reader (mostly for testing purposes)
     * @return      The AI response writer subthread of the endpoint thread
     */
    public AiResponseWriter getAiResponseWriter() {
        return aiResponseWriter;
    }
}

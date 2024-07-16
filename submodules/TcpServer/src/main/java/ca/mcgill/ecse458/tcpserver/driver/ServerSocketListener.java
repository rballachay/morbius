package ca.mcgill.ecse458.tcpserver.driver;

import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.ApplicationContext;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.text.MessageFormat;

import static java.util.Objects.isNull;

/**
 * <p>
 * Threaded class that, once started, listens on the port specified in run() for incoming TCP connection requests from
 * robots.
 * </p>
 *
 * <p>
 * For each incoming connection request, queries the RobotIdService for the next robot ID, creates a RobotThread with it,
 * adds it to the robot thread repository and starts it.
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.repository.RobotRepository ca.mcgill.ecse458.tcpserver.repository.RobotRepository
 * @see ca.mcgill.ecse458.tcpserver.driver.RobotIdService ca.mcgill.ecse458.tcpserver.driver.RobotIdService
 * @author Ze Yuan Fu
 */
@Component
public class ServerSocketListener extends Thread {

    /**
     * Robot thread repository that stores newly created robot threads
     */
    @Autowired
    private RobotRepository robotRepository;

    /**
     * Source of next ID number for new robot threads
     */
    @Autowired
    private RobotIdService robotIdService;

    /**
     * Spring ApplicationContext used to autowire new robot threads
     */
    @Autowired
    private ApplicationContext applicationContext;

    /**
     * Constructor for the server socket listener
     */
    public ServerSocketListener() {
        super("Server Socket Listener");
    }

    public void run() {
        int port = 50050;       // port that the server listens on for TCP connection requests

        System.out.println("starting server socket listener");

        try (ServerSocket serverSocket = new ServerSocket(port)) {
            while (true) {
                // received new connection request
                Socket newSocket = serverSocket.accept();

                // get the next robot ID that is not owned by an existing robot thread
                while (!isNull(robotRepository.getRobotById(robotIdService.get()))) {
                    robotIdService.increment();
                }

                // create robot thread and start the thread
                int currentId = robotIdService.getAndIncrement();
                RobotThread rt = new RobotThread(newSocket, currentId);
                applicationContext.getAutowireCapableBeanFactory().autowireBean(rt);
                rt.start();     // thread will be added to repository after registration by calling addRobotToRepository()
            }
        } catch (IOException e) {
            System.err.println("Could not listen on port " + port);
            throw new RuntimeException(e);
        }
    }

    public void addRobotToRepository(RobotThread rt, int id) {
        robotRepository.addRobotById(id, rt);
        System.out.println(MessageFormat.format("added robot with id {0}",
                String.valueOf(id)));
    }
}

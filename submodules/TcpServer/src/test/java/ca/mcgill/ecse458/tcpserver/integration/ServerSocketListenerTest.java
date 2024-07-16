package ca.mcgill.ecse458.tcpserver.integration;

import ca.mcgill.ecse458.tcpserver.driver.RobotIdService;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import com.sun.management.OperatingSystemMXBean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.lang.management.ManagementFactory;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

@SpringBootTest
public class ServerSocketListenerTest {

    @Autowired
    private RobotRepository robotRepository;
    @Autowired
    private RobotIdService robotIdService;
    private final int numClients = 100, sleepDuration = 3000;

    @AfterEach
    public void clearRepository() {
        for (int i = 1; i <= numClients; i++) {
            robotRepository.deleteRobotById(i);
        }
        robotIdService.resetIdCounter();
    }

    @Test
    public void testAddRobot() {
        try (
                Socket robotSocket = new Socket("localhost", 50050);
                PrintWriter out = new PrintWriter(robotSocket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(robotSocket.getInputStream()));
        ) {
            String serverResponse;

            out.println("HOVERBOT");
            serverResponse = in.readLine();
            assertEquals("set robottype to HOVERBOT", serverResponse);

            Thread.sleep(100);
            assertEquals(1, robotRepository.getRepositorySize());
            assertEquals(1, robotRepository.getRobotById(1).getRobotId());
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            System.exit(1);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    @Test
    public void testAddRobotWithOccupiedNextId() {
        try (
                Socket robotSocket1 = new Socket("localhost", 50050);
                PrintWriter out1 = new PrintWriter(robotSocket1.getOutputStream(), true);
        ) {
            out1.println("HOVERBOT");
            Thread.sleep(200);
            boolean result = robotRepository.updateRobotId(1, 2);
            assertTrue(result);

            Socket robotSocket2 = new Socket("localhost", 50050);
            PrintWriter out2 = new PrintWriter(robotSocket2.getOutputStream(), true);
            out2.println("HOVERBOT");
            Thread.sleep(200);

            assertNull(robotRepository.getRobotById(1));
            assertEquals(2, robotRepository.getRepositorySize());
            assertEquals(2, robotRepository.getRobotById(2).getRobotId());
            assertEquals(3, robotRepository.getRobotById(3).getRobotId());
            out2.close();
            robotSocket2.close();
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            throw new RuntimeException(e);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    @Test
    public void test100RobotThreads() {
        ArrayList<RobotStub> clients = new ArrayList<>();
        OperatingSystemMXBean osBean = ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);

        System.out.printf("starting %d client load test%n", numClients);

        try {
            for (int i = 1; i <= numClients; i++) {
                Socket socket = new Socket("localhost", 50050);
                PrintWriter out = new PrintWriter(socket.getOutputStream(), true);
                BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                out.println("HOVERBOT");
                assertEquals("set robottype to HOVERBOT", in.readLine());

                clients.add(new RobotStub(socket, out, in));
            }

            Thread.sleep(sleepDuration);

            assertEquals(numClients, clients.size());

            System.out.printf("Ran %d clients for %d ms %n", numClients, sleepDuration);
            System.out.printf("JVM CPU load percentage: %.2f%%%n", 100 * osBean.getProcessCpuLoad());
            System.out.printf("Overall CPU load percentage: %.2f%%%n", 100 * osBean.getCpuLoad());
            System.out.printf("Free memory size: %.2f GB%n", osBean.getFreeMemorySize() / Math.pow(2, 30));
        } catch (UnknownHostException e) {
            System.err.println("failed to connect to localhost");
            throw new RuntimeException(e);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        } finally {
            try {
                for (RobotStub rt: clients) {
                    rt.out.close();
                    rt.in.close();
                    rt.socket.close();
                }
            } catch (IOException e) {
                e.printStackTrace();
                throw new RuntimeException(e);
            }
        }
    }

    private static class RobotStub {
        public Socket socket;
        public PrintWriter out;
        public BufferedReader in;

        public RobotStub(Socket socket, PrintWriter pw, BufferedReader br) {
            this.socket = socket;
            this.out = pw;
            this.in = br;
        }
    }
}

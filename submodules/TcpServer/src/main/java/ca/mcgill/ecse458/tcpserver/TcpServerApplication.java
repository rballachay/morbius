package ca.mcgill.ecse458.tcpserver;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
public class TcpServerApplication {

    /**
     * Entry point of the server
     * @param args      Command-line arguments
     */
    public static void main(String[] args) {
        SpringApplication.run(TcpServerApplication.class, args);
    }

}

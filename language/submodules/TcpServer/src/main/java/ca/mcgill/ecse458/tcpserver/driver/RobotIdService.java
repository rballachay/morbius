package ca.mcgill.ecse458.tcpserver.driver;

import org.springframework.stereotype.Component;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * Provides atomically incremented integer robot IDs to the ServerSocketListener.
 *
 * @see ca.mcgill.ecse458.tcpserver.driver.ServerSocketListener ca.mcgill.ecse458.tcpserver.driver.ServerSocketListener
 * @author Ze Yuan Fu
 */
@Component
public class RobotIdService {

    /**
     * AtomicInteger that holds the current value of the ID counter                                                     <br>
     * The counter is initialized to 1 on startup
     */
    private final AtomicInteger idCounter = new AtomicInteger(1);

    /**
     * Returns the current value of the ID counter
     * @return      The current value of the ID counter
     */
    public int get() {
        return idCounter.get();
    }

    /**
     * Returns the current value of the ID counter, and increments its value by 1
     * @return      The current value of the ID counter
     */
    public int getAndIncrement() {
        return idCounter.getAndIncrement();
    }

    /**
     * Increments the value of the ID counter by 1
     */
    public void increment() {
        idCounter.getAndIncrement();
    }

    /**
     * Resets the value of the counter to 1
     */
    public void resetIdCounter() {
        idCounter.set(1);
    }

}

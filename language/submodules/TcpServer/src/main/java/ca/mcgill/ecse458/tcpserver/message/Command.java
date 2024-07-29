package ca.mcgill.ecse458.tcpserver.message;

/**
 * <p>
 * Enumeration class for the possible commands that the robots may perform
 * </p>
 *
 * New entries may be added at the end of the list
 *
 * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
 * @author Ze Yuan Fu
 */
public enum Command {

    /**
     * Move forward
     */
    FORWARD,

    /**
     * Move backward
     */
    BACKWARD,

    /**
     * Turn right
     */
    TURNRIGHT,

    /**
     * Turn left
     */
    TURNLEFT,

    /**
     * Ultrasonic sensor ping
     */
    PING,

    /**
     * Server command, gets a list of the robots currently connected to the server
     */
    GETROBOTS,

    /**
     * Server command, changes a robot's ID
     */
    CHANGEID
}

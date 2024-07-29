package ca.mcgill.ecse458.tcpserver.message;

/**
 * <p>
 * Enumeration class for the possible statuses of a command
 * </p>
 *
 * <p>
 * New entries may be added at the end of the list
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
 * @author Ze Yuan Fu
 */
public enum Status {

    /**
     * The command has been successfully and fully completed by the robot
     */
    SUCCESS,

    /**
     * the robot was unable to successfully complete the command (either no completion or partial completion)
     */
    FAILURE,

    /**
     * The command is being sent to the recipient robot
     */
    DISPATCHED,

    /**
     * The command is invalid (for use with the REST controller)
     */
    INVALID
}

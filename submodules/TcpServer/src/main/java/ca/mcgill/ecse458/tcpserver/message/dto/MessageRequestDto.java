package ca.mcgill.ecse458.tcpserver.message.dto;

/**
 * Data transfer object representing an HTTP request body
 *
 * @see ca.mcgill.ecse458.tcpserver.controller.RestServerController ca.mcgill.ecse458.tcpserver.controller.RestServerController
 * @author Ze Yuan Fu
 */
public class MessageRequestDto {

    /**
     * ID of the robot
     */
    public int id;

    /**
     * Parameter of the command (distance, angle)
     */
    public double param;

    /**
     * Default constructor
     *
     * @param id        ID
     * @param param     Command parameter
     */
    public MessageRequestDto(int id, double param) {
        this.id = id;
        this.param = param;
    }

    /**
     * Returns the ID of the command
     *
     * @return          ID
     */
    public int getId() {
        return id;
    }

    /**
     * Returns the parameter of the command
     *
     * @return          Parameter
     */
    public double getParam() {
        return param;
    }
}

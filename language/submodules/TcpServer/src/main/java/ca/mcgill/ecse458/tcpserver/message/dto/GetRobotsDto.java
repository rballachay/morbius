package ca.mcgill.ecse458.tcpserver.message.dto;

import java.util.List;

/**
 * Data transfer object representing an HTTP response body for the get robots list server
 *
 * @see ca.mcgill.ecse458.tcpserver.message.dto.RobotDto ca.mcgill.ecse458.tcpserver.message.dto.RobotDto
 * @author Ze Yuan Fu
 */
public class GetRobotsDto {

    /**
     * List of RobotDto objects
     */
    public List<RobotDto> robots;

    /**
     * Default constructor for Jackson
     */
    public GetRobotsDto() {}

    /**
     * Constructor that initializes a GetRobotsDto object using a list of RobotDto objects
     *
     * @param robots        List of RobotDtos
     */
    public GetRobotsDto(List<RobotDto> robots) {
        this.robots = robots;
    }

    /**
     * Returns the list of RobotDtos
     *
     * @return              List of RobotDtos
     */
    public List<RobotDto> getRobots() {
        return robots;
    }
}

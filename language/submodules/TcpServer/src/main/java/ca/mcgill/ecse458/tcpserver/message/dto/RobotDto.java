package ca.mcgill.ecse458.tcpserver.message.dto;

import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import ca.mcgill.ecse458.tcpserver.robot.RobotType;

/**
 * Data transfer object representing a RobotThread object. Stored in a GetRobotDto object during transport.
 *
 * @see ca.mcgill.ecse458.tcpserver.message.dto.GetRobotsDto ca.mcgill.ecse458.tcpserver.message.dto.GetRobotsDto
 * @author Ze Yuan Fu
 */
public class RobotDto {

    /**
     * ID of the represented robot thread
     */
    public int id;

    /**
     * Type of the represented robot thread
     */
    public RobotType robotType;

    /**
     * Default constructor
     *
     * @param id            ID of robot
     * @param robotType     Type of robot
     */
    public RobotDto(int id, RobotType robotType) {
        this.id = id;
        this.robotType = robotType;
    }

    /**
     * Constructor that initializes a RobotDto using a RobotThread object
     * @param rt            RobotThread to be represented
     */
    public RobotDto(RobotThread rt) {
        this.id = rt.getRobotId();
        this.robotType = rt.getRobotType();
    }

    @Override
    public boolean equals(Object o) {
        if (o == null) {
            return false;
        }

        if (o.getClass() != this.getClass()) {
            return false;
        }

        final RobotDto other = (RobotDto) o;

        return other.getId() == this.id && other.getRobotType() == this.robotType;
    }

    /**
     * Returns the ID of the RobotDto
     *
     * @return      ID of robot
     */
    public int getId() {
        return id;
    }

    /**
     * Returns the type of the RobotDto
     *
     * @return      Type of robot
     */
    public RobotType getRobotType() {
        return robotType;
    }
}

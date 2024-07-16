package ca.mcgill.ecse458.tcpserver.controller;

import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.message.dto.GetRobotsDto;
import ca.mcgill.ecse458.tcpserver.message.dto.MessageRequestDto;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import static java.util.Objects.isNull;

/**
 * <p>
 * REST controller that allows users to issue commands to robots by sending HTTP requests to the server.
 * </p>
 *
 * <p>
 * Each command has its own HTTP endpoint:                                                                              <br>
 * - forward:           POST    /forward                                                                                <br>
 * - backward:          POST    /backward                                                                               <br>
 * - turn right:        POST    /turnright                                                                              <br>
 * - turn left:         POST    /turnleft                                                                               <br>
 * - ping:              POST    /ping                                                                                   <br>
 * - get robot list:    GET     /robots                                                                                 <br>
 * - change robot id:   POST    /changeid                                                                               <br>
 * </p>
 *
 * <p>
 * Each command requires in the HTTP request body the ID of the recipient robot, and the parameter specific to the
 * command being executed (distance for movement, angle for turns - except the ping and getrobots commands, which do not
 * require a parameter). These values should be encoded in the request body (with the "application/json" header) as such:
 * </p>
 *
 * <p>
 * {"id":1}                     when a parameter is not required                                                        <br>
 * {"id":1, "param":123.456}    when a parameter is required                                                            <br>
 * </p>
 *
 * <p>
 * The controller will return a Message response with the appropriate ID, command and parameter along with the
 * DISPATCHED status. If the robot ID is invalid, the controller will return an HTTP 400 response instead.
 * </p>
 *
 * <p>
 * Since the controller does not support waiting for robot responses, the response will be displayed in the console.
 * Server command responses will be returned in the HTTP response body.
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
 * @see MessageRequestDto ca.mcgill.ecse458.tcpserver.message.dto.MessageRequestDto
 * @author Ze Yuan Fu
 */
@CrossOrigin
@RestController
public class RestServerController {

    /**
     * Reference to the server's robot repository, used to retrieve robot threads to send commands to
     * @see ca.mcgill.ecse458.tcpserver.repository.RobotRepository ca.mcgill.ecse458.tcpserver.repository.RobotRepository
     */
    @Autowired
    RobotRepository robotRepository;

    /**
     * Returns a list of the robots currently connected to the server
     *
     * @HTTPmethod          GET
     * @URL                 /robots
     * @return              List of robots connected to the server
     */
    @GetMapping(value = "/robots")
    @ResponseBody
    public GetRobotsDto getRobotsList() {
        return new GetRobotsDto(robotRepository.getRobotDtos());
    }

    /**
     * Sends a move forward command to a robot
     *
     * @HTTPmethod          POST
     * @URL                 /forward
     * @param request       Request DTO containing the ID and distance parameters
     * @return              A Message representing the command
     */
    @PostMapping(value = "/forward")
    @ResponseBody
    public Message forwardCommand(@RequestBody MessageRequestDto request) {
        if (!isNull(robotRepository.getRobotById(request.getId()))) {
            Message command = new Message.MessageBuilder(request.getId(), Command.FORWARD, Status.DISPATCHED)
                    .floatData(new double[]{request.getParam()}).build();
            robotRepository.getRobotById(request.getId()).putMessage(command);

            return command;
        } else {
            throw new RobotCommandException(HttpStatus.BAD_REQUEST, "No robot with the given ID exists");
        }
    }

    /**
     * Sends a move backward command to a robot
     *
     * @HTTPmethod          POST
     * @URL                 /backward
     * @param request       Request DTO containing the ID and distance parameters
     * @return              A Message representing the command
     */
    @PostMapping(value = "/backward")
    @ResponseBody
    public Message backwardCommand(@RequestBody MessageRequestDto request) {
        if (!isNull(robotRepository.getRobotById(request.getId()))) {
            Message command = new Message.MessageBuilder(request.getId(), Command.BACKWARD, Status.DISPATCHED)
                    .floatData(new double[]{request.getParam()}).build();
            robotRepository.getRobotById(request.getId()).putMessage(command);

            return command;
        } else {
            throw new RobotCommandException(HttpStatus.BAD_REQUEST, "No robot with the given ID exists");
        }
    }

    /**
     * Sends a turn right command to a robot
     *
     * @HTTPmethod          POST
     * @URL                 /turnright
     * @param request       Request DTO containing the ID and angle parameters
     * @return              A Message representing the command
     */
    @PostMapping(value = "/turnright")
    @ResponseBody
    public Message turnRightCommand(@RequestBody MessageRequestDto request) {
        if (!isNull(robotRepository.getRobotById(request.getId()))) {
            Message command = new Message.MessageBuilder(request.getId(), Command.TURNRIGHT, Status.DISPATCHED)
                    .floatData(new double[]{request.getParam()}).build();
            robotRepository.getRobotById(request.getId()).putMessage(command);

            return command;
        } else {
            throw new RobotCommandException(HttpStatus.BAD_REQUEST, "No robot with the given ID exists");
        }
    }

    /**
     * Sends a turn left command to a robot
     *
     * @HTTPmethod          POST
     * @URL                 /turnleft
     * @param request       Request DTO containing the ID and angle parameters
     * @return              A Message representing the command
     */
    @PostMapping(value = "/turnleft")
    @ResponseBody
    public Message turnLeftCommand(@RequestBody MessageRequestDto request) {
        if (!isNull(robotRepository.getRobotById(request.getId()))) {
            Message command = new Message.MessageBuilder(request.getId(), Command.TURNLEFT, Status.DISPATCHED)
                    .floatData(new double[]{request.getParam()}).build();
            robotRepository.getRobotById(request.getId()).putMessage(command);

            return command;
        } else {
            throw new RobotCommandException(HttpStatus.BAD_REQUEST, "No robot with the given ID exists");
        }
    }

    /**
     * Sends a ping command to a robot
     *
     * @HTTPmethod          POST
     * @URL                 /ping
     * @param request       Request DTO containing the ID parameter
     * @return              A Message representing the command
     */
    @PostMapping(value = "/ping")
    @ResponseBody
    public Message pingCommand(@RequestBody MessageRequestDto request) {
        if (!isNull(robotRepository.getRobotById(request.getId()))) {
            Message command = new Message.MessageBuilder(request.getId(), Command.PING, Status.DISPATCHED).build();
            robotRepository.getRobotById(request.getId()).putMessage(command);

            return command;
        } else {
            throw new RobotCommandException(HttpStatus.BAD_REQUEST, "No robot with the given ID exists");
        }
    }

    /**
     * Changes the ID of a robot
     *
     * @HTTPmethod          POST
     * @URL                 /changeid
     * @param request       Request DTO containing the old ID and new ID parameters
     * @return              A Message representing the command
     */
    @PostMapping(value = "/changeid")
    @ResponseBody
    public Message changeRobotID(@RequestBody MessageRequestDto request) {
        if (!isNull(robotRepository.getRobotById(request.getId()))) {
            if (robotRepository.updateRobotId(request.getId(), (int) request.getParam())) {
                return new Message.MessageBuilder(request.getId(), Command.CHANGEID, Status.SUCCESS).build();
            } else {
                throw new RobotCommandException(HttpStatus.BAD_REQUEST,
                        "The repository already contains a robot with the given new ID");
            }
        } else {
            throw new RobotCommandException(HttpStatus.BAD_REQUEST, "No robot with the given ID exists");
        }
    }
}

/**
 * Custom exception that is thrown when a robot command parameter is invalid
 *
 * @author Ze Yuan Fu
 */
class RobotCommandException extends RuntimeException {

    /**
     * HTTP status code of the response
     */
    private final HttpStatus httpStatus;

    /**
     * Default constructor with a status code and a message
     *
     * @param httpStatus        Status code of the response
     * @param message           Message of the response
     */
    public RobotCommandException(HttpStatus httpStatus, String message) {
        super(message);
        this.httpStatus = httpStatus;
    }

    /**
     * Returns the status code of the exception
     *
     * @return              Status code of the exception
     */
    public HttpStatus getHttpStatus() {
        return httpStatus;
    }
}

/**
 * Exception handler for the REST controller
 *
 * @author Ze Yuan Fu
 */
@ControllerAdvice
class ExceptionHandlerAdvice {

    /**
     * Handles RobotCommandExceptions by returning an HTTP response with the exception's status code and message
     *
     * @param e         RobotCommandException to handle
     * @return          ResponseEntity with status code and message
     */
    @ExceptionHandler(RobotCommandException.class)
    public ResponseEntity<String> handleException(RobotCommandException e) {
        return ResponseEntity.status(e.getHttpStatus()).body(e.getMessage());
    }
}


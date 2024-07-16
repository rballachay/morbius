package ca.mcgill.ecse458.tcpserver.message;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.Arrays;
import java.util.Objects;

import static java.util.Objects.isNull;

/**
 * <p>
 * Message class used for communication between the user interface, AI TCP endpoint, REST server controller and robot threads.
 * </p>
 *
 * <p>
 * Contains information about the type of command, the int and float parameters of the command, the result of the
 * command (distance travelled, angle turned, etc.), the status of the command (success or failure) and any textual
 * messages the robot may wish to send the server. The latter three properties are determined by the robot and
 * transmitted to the robot thread on the completion of the command.
 * </p>
 *
 * <p>
 * When a message is in the DISPATCHED state, its result field will be empty (set to 0.0). The result field will be
 * filled by the robot before the message is sent back to the server.
 * </p>
 *
 * <p>
 * This class can easily be extended with additional optional fields should the need for them arise. You will need to
 * add the following:                                                                                                   <br>
 * - a field in the Message class                                                                                       <br>
 * - a parameter and its corresponding line in the public @JsonCreator constructor for Jackson                          <br>
 * - a parameter and its corresponding line in the private Message constructor                                          <br>
 * - a getter for the field in the Message class                                                                        <br>
 * - a condition in the .equals() override return statement                                                             <br>
 * - a field in the Message.MessageBuilder class                                                                        <br>
 * - a setter method in the Message.MessageBuilder class
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.message.Command ca.mcgill.ecse458.tcpserver.message.Command
 * @see ca.mcgill.ecse458.tcpserver.message.Status ca.mcgill.ecse458.tcpserver.message.Status
 * @author Ze Yuan Fu
 */
public class Message {

    /**
     * ID of the robot which will execute this command (mandatory)
     */
    private final int id;

    /**
     * Type of command to be executed (mandatory)
     * @see ca.mcgill.ecse458.tcpserver.message.Command ca.mcgill.ecse458.tcpserver.message.Command
     */
    private final Command command;

    /**
     * Status of the command (mandatory)
     * @see ca.mcgill.ecse458.tcpserver.message.Status ca.mcgill.ecse458.tcpserver.message.Status
     */
    private final Status status;

    /**
     * Array of integer command parameters
     */
    private final int[] intData;

    /**
     * Array of float command parameters
     */
    private final double[] floatData;

    /**
     * Result of the command (distance moved, angle turned, distance pinged - is determined after the command has finished executing)
     */
    private final double result;

    /**
     * Textual message that the robot may send to the server
     */
    private final String text;

    /**
     * Default constructor for Jackson, is not explicitly invoked by the server code
     */
    @JsonCreator
    public Message( @JsonProperty("id") int id,
                    @JsonProperty("command") Command command,
                    @JsonProperty("status") Status status,
                    @JsonProperty("intData") int[] intData,
                    @JsonProperty("floatData") double[] floatData,
                    @JsonProperty("result") double result,
                    @JsonProperty("text") String text) {
        super();
        this.id = id;
        this.command = command;
        this.status = status;
        this.intData = intData;
        this.floatData = floatData;
        this.result = result;
        this.text = text;
    }

    /**
     * Constructor for builder
     * @param builder       MessageBuilder containing the information required to build this message
     */
    private Message(MessageBuilder builder) {
        this.id = builder.id;
        this.command = builder.command;
        this.status = builder.status;
        this.intData = !isNull(builder.intData) ? builder.intData : new int[] {};
        this.floatData = !isNull(builder.floatData) ? builder.floatData : new double[] {};
        this.result = builder.result;
        this.text = !isNull(builder.text) ? builder.text : "";
    }

    /**
     * Gets the ID of the message
     * @return      ID of the message
     */
    public int getId() {
        return id;
    }

    /**
     * Gets the command of the message
     * @return      Command of the message
     */
    public Command getCommand() {
        return command;
    }

    /**
     * Gets the status of the command
     * @return      Status of the command
     */
    public Status getStatus() {
        return status;
    }

    /**
     * Gets the integer parameters of the command
     * @return      Integer parameters of the command
     */
    public int[] getIntData() {
        return intData;
    }

    /**
     * Gets the float parameters of the command
     * @return      Float parameters of the command
     */
    public double[] getFloatData() {
        return floatData;
    }

    /**
     * Gets the result of the command
     * @return      Result of the command
     */
    public double getResult() {
        return result;
    }

    /**
     * Gets the message text of the command
     * @return      Message text of the command
     */
    public String getText() {
        return text;
    }

    @Override
    public boolean equals(Object o) {
        if (o == null) {
            return false;
        }

        if (o.getClass() != this.getClass()) {
            return false;
        }

        final Message other = (Message) o;

        return other.getId() == id && other.getCommand() == command && other.getStatus() == status &&
                Arrays.equals(other.getIntData(), intData) && Arrays.equals(other.getFloatData(), floatData) &&
                other.getResult() == result && Objects.equals(other.getText(), text);
    }

    /**
     * Builder class for Messages
     */
    public static class MessageBuilder {

        private final int id;
        private final Command command;
        private final Status status;
        private int[] intData;
        private double[] floatData;
        private double result;
        private String text;

        /**
         * Constructor for the Message builder, takes as parameter the three mandatory fields
         * @param id            ID of the recipient
         * @param command       Type of command
         * @param status        Status of the command
         */
        public MessageBuilder(int id, Command command, Status status) {
            this.id = id;
            this.command = command;
            this.status = status;
        }

        /**
         * Adds integer parameters to the message
         * @param intData       Integer parameters of the command
         * @return              A MessageBuilder object with integer parameters
         */
        public MessageBuilder intData(int[] intData) {
            this.intData = intData;
            return this;
        }

        /**
         * Adds float parameters to the message
         * @param floatData     Float parameters of the command
         * @return              A MessageBuilder object with float parameters
         */
        public MessageBuilder floatData(double[] floatData) {
            this.floatData = floatData;
            return this;
        }

        /**
         * Adds the result to the message
         * @param result        Result of the command
         * @return              A MessageBuilder object with a set result
         */
        public MessageBuilder result(Double result) {
            this.result = result;
            return this;
        }

        /**
         * Adds a textual message to the message
         * @param text          Text message of the command
         * @return              A MessageBuilder object with a text message
         */
        public MessageBuilder text(String text) {
            this.text = text;
            return this;
        }

        /**
         * Builds the Message
         * @return              A Message object with the values of the fields of its builder
         */
        public Message build() {
            return new Message(this);
        }
    }

}

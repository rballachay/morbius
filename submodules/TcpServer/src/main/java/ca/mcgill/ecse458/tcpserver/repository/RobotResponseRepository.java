package ca.mcgill.ecse458.tcpserver.repository;

import ca.mcgill.ecse458.tcpserver.message.Message;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Optional;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * <p>
 * Repository that stores the responses received from robots in a deque.
 * </p>
 *
 * <p>
 * The contents of the deque are modified using a lock and wait-notify methods for thread-safety.
 * </p>
 *
 * <p>
 * Currently only used with the AI response writer and for testing purposes but may be extended for additional
 * functionality.
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.message.Message ca.mcgill.ecse458.tcpserver.message.Message
 * @author Ze Yuan Fu
 */
@Component
public class RobotResponseRepository {

    /**
     * Robot thread repository used to retrieve robots in order to access their Conditions
     * @see ca.mcgill.ecse458.tcpserver.robot.RobotThread ca.mcgill.ecse458.tcpserver.robot.RobotThread
     * @see ca.mcgill.ecse458.tcpserver.repository.RobotRepository ca.mcgill.ecse458.tcpserver.repository.RobotRepository
     */
    @Autowired
    private RobotRepository robotRepository;

    /**
     * Deque that stores the robot responses
     */
    private final Deque<Message> messageQueue = new ArrayDeque<>();

    /**
     * Lock to maintain thread safety when accessing the Deque
     */
    private final Lock lock = new ReentrantLock();

    /**
     * Condition for when the repository is not empty
     */
    private final Condition notEmpty = lock.newCondition();

    /**
     * Adds a message to the deque in the last position
     * @param message       Message to add to the repository
     */
    public void putMessage(Message message) {
        lock.lock();
        try {
            messageQueue.addLast(message);
            notEmpty.signal();
        } finally {
            lock.unlock();
        }
    }

    /**
     * Removes and returns the first message in the repository. If the repository is empty, waits until a message is
     * added, then retrieves it.
     * @return              First message in the repository
     */
    public Message removeMessage() {
        lock.lock();
        try {
            while(messageQueue.isEmpty()) {
                notEmpty.await();
            }

            return messageQueue.pop();
        } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        } finally {
            lock.unlock();
        }
    }

    /**
     * Helper function to determine if the queue contains a message with the specified ID
     * @param id            ID of robot
     * @return              True if the repository contains a response from the robot thread with the given ID, false otherwise
     */
    private boolean hasMessageById(int id) {
        Optional<Message> message = messageQueue.stream().filter(m -> m.getId() == id).findFirst();
        return message.isPresent();
    }

    /**
     * Removes and returns the first message from the robot with the specified ID. If the repository does not contain a
     * such message, waits until the robot with the specified ID has sent a response to the repository, then retrieves it
     * @param id            ID of robot
     * @return              First message from the robot with the specified ID
     */
    public Message getResponseById(int id) {
        lock.lock();
        try {
            while(!hasMessageById(id)) {
                robotRepository.getRobotById(id).getHasResponse().await();
            }

            Message response = messageQueue.stream().filter(m -> m.getId() == id).findFirst().get();
            messageQueue.remove(response);
            return response;
        } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        } finally {
            lock.unlock();
        }
    }

    /**
     * Returns the number of messages in the repository
     * @return              Number of messages in the repository
     */
    public int getQueueSize() {
        lock.lock();
        try {
            return messageQueue.size();
        } finally {
            lock.unlock();
        }
    }

    /**
     * Empties the repository (mostly used for testing purposes)
     */
    public void emptyQueue() {
        lock.lock();
        try {
            messageQueue.clear();
        } finally {
            lock.unlock();
        }
    }
}

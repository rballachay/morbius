package ca.mcgill.ecse458.tcpserver.repository;

import ca.mcgill.ecse458.tcpserver.message.dto.RobotDto;
import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * <p>
 * Repository that stores references to every robot thread using an ID:Thread mapping in a HashMap. Two RobotThreads may
 * not be mapped to the same ID.
 * </p>
 *
 * <p>
 * The contents of the HashMap are modified using synchronized methods for thread-safety.
 * </p>
 *
 * @see ca.mcgill.ecse458.tcpserver.robot.RobotThread ca.mcgill.ecse458.tcpserver.robot.RobotThread
 * @author Ze Yuan Fu
 */
@Component
public class RobotRepository {

    /**
     * HashMap storing Robot threads
     * @see ca.mcgill.ecse458.tcpserver.robot.RobotThread ca.mcgill.ecse458.tcpserver.robot.RobotThread
     */
    private final HashMap<Integer, RobotThread> repository = new HashMap<>();

    /**
     * Returns the RobotThread associated with the given ID, or null if no RobotThread exists for the given ID
     * @param id            ID of the robot thread to retrieve
     * @return              RobotThread with the given ID, or null if no RobotThread exists for the given ID
     */
    public synchronized RobotThread getRobotById(int id) {
        return repository.get(id);
    }

    /**
     * Adds the given RobotThread to the repository if the given ID is not already mapped to a RobotThread
     * @param id            ID to map the new RobotThread to
     * @param robotThread   RobotThread to add to the repository
     * @return              True if the operation succeeded (repository does not contain a RobotThread with the given ID), false otherwise
     */
    public synchronized boolean addRobotById(int id, RobotThread robotThread) {
        if (!repository.containsKey(id)) {
            repository.put(id, robotThread);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Updates the ID of a RobotThread to the given new ID if the new ID is not already mapped to a RobotThread
     * @param oldId         ID of the RobotThread whose ID is updated
     * @param newId         ID to update the RobotThread with
     * @return              True if the operation succeeded (repository does not contain a RobotThread with the given new ID), false otherwise
     */
    public synchronized boolean updateRobotId(int oldId, int newId) {
        if (repository.containsKey(oldId) && !repository.containsKey(newId)) {
            repository.get(oldId).setRobotId(newId);
            repository.put(newId, repository.remove(oldId));
            return true;
        } else {
            return false;
        }
    }

    /**
     * Deletes the RobotThread with the given ID if the mapping exists
     * @param id            ID of the RobotThread to delete
     * @return              True if the operation succeeded (repository contains a RobotThread with the given ID), false otherwise
     */
    public synchronized boolean deleteRobotById(int id) {
        if (repository.containsKey(id)) {
            repository.remove(id);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns the number of robots currently in the repository
     * @return              Number of robots currently in the repository
     */
    public synchronized int getRepositorySize() {
        return repository.size();
    }

    /**
     * Returns a list of RobotThreads currently in the repository
     * @return              ArrayList of RobotThreads currently in the repository
     */
    public synchronized ArrayList<RobotThread> getRobots() {
        return new ArrayList<>(repository.values());
    }

    /**
     * Returns a list of Robot DTOs currently in the repository
     * @return              ArrayList of Robot DTOs currently in the repository
     */
    public synchronized ArrayList<RobotDto> getRobotDtos() {
        ArrayList<RobotDto> results = new ArrayList<>();

        for (RobotThread rt: repository.values()) {
            results.add(new RobotDto(rt));
        }

        return results;
    }
}

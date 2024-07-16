file://<WORKSPACE>/submodules/TcpServer/src/main/java/ca/mcgill/ecse458/tcpserver/driver/UserInterface.java
### java.util.NoSuchElementException: next on empty iterator

occurred in the presentation compiler.

presentation compiler configuration:
Scala version: 3.3.3
Classpath:
<HOME>/Library/Caches/Coursier/v1/https/repo1.maven.org/maven2/org/scala-lang/scala3-library_3/3.3.3/scala3-library_3-3.3.3.jar [exists ], <HOME>/Library/Caches/Coursier/v1/https/repo1.maven.org/maven2/org/scala-lang/scala-library/2.13.12/scala-library-2.13.12.jar [exists ]
Options:



action parameters:
offset: 6410
uri: file://<WORKSPACE>/submodules/TcpServer/src/main/java/ca/mcgill/ecse458/tcpserver/driver/UserInterface.java
text:
```scala
package ca.mcgill.ecse458.tcpserver.driver;

import ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint;
import ca.mcgill.ecse458.tcpserver.message.Command;
import ca.mcgill.ecse458.tcpserver.message.Message;
import ca.mcgill.ecse458.tcpserver.message.Status;
import ca.mcgill.ecse458.tcpserver.repository.RobotRepository;
import ca.mcgill.ecse458.tcpserver.robot.RobotThread;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.CommandLineRunner;
import org.springframework.context.ApplicationContext;
import org.springframework.stereotype.Component;

import java.text.MessageFormat;
import java.util.Objects;
import java.util.Scanner;

import static java.util.Objects.isNull;

/**
 * <p>
 * Simple choice-based user interface to issue commands to robots.
 * </p>
 *
 * Note: this class is not unit- or integration-tested.
 *
 * @author Ze Yuan Fu
 */
@Component
public class UserInterface implements CommandLineRunner {

    /**
     * Reference to the server's robot repository, used to retrieve robot threads to send commands to
     */
    @Autowired
    private RobotRepository robotRepository;

    /**
     * Spring ApplicationContext used to autowire the subthreads created by this class
     */
    @Autowired
    private ApplicationContext applicationContext;

    @Override
    public void run(String... args) {
        System.out.println("commandline started");

        ServerSocketListener ssl = new ServerSocketListener();
        applicationContext.getAutowireCapableBeanFactory().autowireBean(ssl);
        ssl.start();

        if (!Objects.equals(System.getenv("server_test_mode"), "true")) {
            AiTcpEndpoint ate = new AiTcpEndpoint();
            applicationContext.getAutowireCapableBeanFactory().autowireBean(ate);
            ate.start();
        }

        if (!Objects.equals(System.getenv("server_test_mode"), "true")) {
            Scanner scanner = new Scanner(System.in);
            while(true) {
                int id, command;
                double param;

                System.out.println("""
                    Select a command:
                    1 - Move forward
                    2 - Move backward
                    3 - Turn left
                    4 - Turn right
                    5 - Ping
                    6 - Change robot id
                    7 - List robots""");

                try {
                    command = Integer.parseInt(scanner.nextLine());
                } catch (NumberFormatException e) {
                    System.out.println("Not a valid integer command, please try again ");
                    continue;
                }

                if (command != 7) {
                    System.out.println("Enter a robot id: ");

                    try {
                        id = Integer.parseInt(scanner.nextLine());
                    } catch (NumberFormatException e) {
                        System.out.println("Not a valid integer id, please try again ");
                        continue;
                    }
                } else {
                    id = 0;
                }

                switch (command) {
                    case 1 -> {
                        System.out.println("Enter a distance");
                        try {
                            param = Double.parseDouble(scanner.nextLine());
                        } catch (NumberFormatException e) {
                            System.out.println("Not a valid double, please try again ");
                            continue;
                        }
                        if (!isNull(robotRepository.getRobotById(id))) {
                            robotRepository.getRobotById(id).putMessage(
                                    new Message.MessageBuilder(id, Command.FORWARD, Status.DISPATCHED).
                                            floatData(new double[] {param}).build());
                        }
                    }
                    case 2 -> {
                        System.out.println("Enter a distance");
                        try {
                            param = Double.parseDouble(scanner.nextLine());
                        } catch (NumberFormatException e) {
                            System.out.println("Not a valid double, please try again ");
                            continue;
                        }
                        if (!isNull(robotRepository.getRobotById(id))) {
                            robotRepository.getRobotById(id).putMessage(
                                    new Message.MessageBuilder(id, Command.BACKWARD, Status.DISPATCHED).
                                            floatData(new double[] {param}).build());
                        }
                    }
                    case 3 -> {
                        System.out.println("Enter an angle");
                        try {
                            param = Double.parseDouble(scanner.nextLine());
                        } catch (NumberFormatException e) {
                            System.out.println("Not a valid double, please try again ");
                            continue;
                        }
                        if (!isNull(robotRepository.getRobotById(id))) {
                            robotRepository.getRobotById(id).putMessage(
                                    new Message.MessageBuilder(id, Command.TURNLEFT, Status.DISPATCHED).
                                            floatData(new double[] {param}).build());
                        }
                    }
                    case 4 -> {
                        System.out.println("Enter an angle");
                        try {
                            param = Double.parseDouble(scanner.nextLine());
                        } catch (NumberFormatException e) {
                            System.out.println("Not a valid double, please try again ");
                            continue;
                        }
                        if (!isNull(robotRepository.getRobotById(id))) {
                            robotRepository.getRobotById(id).putMessage(
                                    new Message.MessageBuilder(id, Command.TURNRIGHT, Status.DISPATCHED).
                                            floatData(new double[] {param}).build());
                        }
                    }
                    case 5 -> {
                       @@ System.out.println("pinging");
                        if (!isNull(robotRepository.getRobotById(id))) {
                            robotRepository.getRobotById(id).putMessage(
                                    new Message.MessageBuilder(id, Command.PING, Status.DISPATCHED).build());
                        }
                    }
                    case 6 -> {
                        System.out.println("Enter an id");
                        try {
                            param = Double.parseDouble(scanner.nextLine());
                        } catch (NumberFormatException e) {
                            System.out.println("Not a valid double, please try again ");
                            continue;
                        }
                        if (!isNull(robotRepository.getRobotById(id))) {
                            robotRepository.updateRobotId(id, (int) param);
                            System.out.println(MessageFormat.format("updated robot with id {0} to {1}", id, (int) param));
                        }
                    }
                    case 7 -> {
                        System.out.println("listing robots");
                        for (RobotThread rt : robotRepository.getRobots()) {
                            System.out.println(MessageFormat.format("id {0}: robot {1}", rt.getRobotId(), rt.getRobotType()));
                        }
                    }
                }
            }
        }
    }
}

```



#### Error stacktrace:

```
scala.collection.Iterator$$anon$19.next(Iterator.scala:973)
	scala.collection.Iterator$$anon$19.next(Iterator.scala:971)
	scala.collection.mutable.MutationTracker$CheckedIterator.next(MutationTracker.scala:76)
	scala.collection.IterableOps.head(Iterable.scala:222)
	scala.collection.IterableOps.head$(Iterable.scala:222)
	scala.collection.AbstractIterable.head(Iterable.scala:933)
	dotty.tools.dotc.interactive.InteractiveDriver.run(InteractiveDriver.scala:168)
	scala.meta.internal.pc.MetalsDriver.run(MetalsDriver.scala:45)
	scala.meta.internal.pc.HoverProvider$.hover(HoverProvider.scala:36)
	scala.meta.internal.pc.ScalaPresentationCompiler.hover$$anonfun$1(ScalaPresentationCompiler.scala:380)
```
#### Short summary: 

java.util.NoSuchElementException: next on empty iterator
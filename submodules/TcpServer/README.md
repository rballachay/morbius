# Prometheus Swarm TCP server

## Table of contents
<!-- TOC -->
* [About this project](#about-this-project)
  * [Built with](#built-with)
* [Getting started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
  * [Server Ports](#server-ports)
  * [Starting the server](#starting-the-server)
  * [Sending commands](#sending-commands)
    * [Integrated command line interface](#integrated-command-line-interface)
    * [Python client](#python-client)
    * [META AI endpoint](#meta-ai-endpoint)
* [Testing](#testing)
  * [Automatic testing](#automatic-testing)
  * [Manual testing](#manual-testing)
* [Server API Reference](#server-api-reference)
  * [Robots](#robots)
  * [META AI](#meta-ai)
  * [REST endpoint](#rest-endpoint)
* [Javadoc](#javadoc)
* [Roadmap](#roadmap)
* [Appendix](#appendix)
  * [Sample API usage](#sample-api-usage)
    * [Robot](#robot)
    * [META AI](#meta-ai)
<!-- TOC -->

## About this project

Java server enabling user-robot communications using TCP connections. Commands are entered via the command line or using
the Python REST client, processed by the server and sent to the recipient robot, which executes it and sends back a 
response, which is processed again and made viewable to the user.

Optionally supports a direct connection to the Prometheus META AI for direct real-time command and response streaming.

### Built with

- Java
- Spring Framework
- Gradle
- IntelliJ IDEA

[Back to the top](#table-of-contents)


## Getting started

To get the project up and running, follow the following steps:

### Prerequisites

- [Java](https://www.oracle.com/java/technologies/javase/jdk17-archive-downloads.html) (version 17 is recommended)
- [Python](https://www.python.org/downloads/) (optional, to use the Python REST client)
- [NetCat](https://nmap.org/download.html) (optional, for manual testing)
- Your favorite IDE (IntelliJ IDEA is recommended for its native Gradle integration ~~but no judgement if you like Eclipse better~~)
- A Wi-Fi network (to use the server or perform multi-device testing)

### Installation

1. Clone the repo 
    ```shell
    git clone git@gitlab.cs.mcgill.ca:jvybihal/prometheus-swam.git
    ```
   
2. Navigate to the `/TcpServer` directory
    ```shell
    cd prometheus-swarm/TcpServer/
    ```

3. (Optional) Install the Python REST client's dependencies
   ```shell
   pip install -r requirements.txt
   ```
   
4. Build the project with Gradle (use `./gradlew.bat` on Windows)
   ```shell
   ./gradlew build
   ```

[Back to the top](#table-of-contents)


## Usage

### Server Ports

The server listens for connections from robots on port `50050` and from the META AI on port `50049`; make sure to direct
all connection attempts to those two ports. These values may be changed in `ca.mcgill.ecse458.tcpserver.driver.ServerSocketListener`
and `ca.mcgill.ecse458.tcpserver.controller.AiTcpEndpoint`, respectively.

The REST controller listens for HTTP requests on `8080`. This value may be changed in the file 
`prometheus-swarm/TcpServer/src/main/resources/application.properties`.

### Starting the server

1. Run the project with Gradle
   ```shell
   ./gradlew bootrun
   ```
   
2. Verify that the following console output is present:
   ```shell
   commandline started
   starting server socket listener
   starting AI TCP endpoint
   ```

After this point, no more user input is required and robots may connect.

### Sending commands

Several methods are provided to send commands to the server

#### Integrated command line interface

Follow the terminal prompts: select a command by entering the number corresponding to the desired command, then enter IDs 
and parameters (if applicable). Robot responses will be printed to the server console when received.

#### Python client

Run `python client.py` and follow the terminal prompts: select a command by entering the number corresponding to the 
desired command, then enter IDs and parameters. Robot responses will be printed to the server console when received.

#### META AI endpoint

See the Manual testing section, step 3, for instructions to connect to the AI server endpoint, and the META AI section
of the server API documentation below for the API usage. Robot responses are sent to the AI client as soon as they are 
received.

[Back to the top](#table-of-contents)


## Testing

This section will describe the procedure to follow to perform automatic and manual testing.

### Automatic testing

To run the unit and integration test suite:

1. Run the tests with Gradle
   ```shell
   ./gradlew test
   ```

### Manual testing

1. Start the server
   ```shell
   ./gradlew bootrun
   ```

Adding robot stubs:

2. In a new terminal window, start a connection to `localhost:50050` with NetCat
   ```shell
   ncat localhost 50050
   ```
   
   Verify that the server has processed the robot's connection attempt in the server console
   
   ```shell
   added robot with id 1
   ```

   Enter a robot type(`HOVERBOT` or `BOEBOT`) in the NetCat terminal and verify the server's confirmation

   ```shell
   HOVERBOT
   set robottype to HOVERBOT
   ```

   ```shell
   BOEBOT
   set robottype to BOEBOT
   ```
   
   Please note that no commands will be sent to the robots until the robot type is confirmed.

   Repeat this step to obtain the desired number of robots

(Optional) Adding an AI stub:

3. In a new terminal window, start a connection to `localhost:50049` with NetCat
   ```shell
   ncat localhost 50049
   ```

   Verify that the server has processed the AI's connection attempt in the server console

   ```shell
   AI connection established, starting subthreads
   starting AI command reader                 
   starting AI response writer
   ```

Commands may now be sent to the robots using the command line interface, Python REST client or the AI NetCat terminal.

To send a command from the AI NetCat terminal, or a response from a robot NetCat terminal, simply type (or paste) the 
message in the terminal and press enter.

See the API section below for details about sending commands using the AI endpoint, and responding to commands from the 
robot stubs.

[Back to the top](#table-of-contents)


## Server API Reference

This section will describe the API of the server for robots, the META AI and the REST controller.

### Robots

After establishing a connection to the server, robots are expected to send a string containing their robot type to the 
server, which will return a confirmation response:

```shell
HOVERBOT
set robottype to HOVERBOT
```

```shell
BOEBOT
set robottype to BOEBOT
```

The server will then start sending commands to the robot.

Command messages are sent to robots as JSON-formatted strings in the following format:

```json
{"id":1,"command":"FORWARD","status":"DISPATCHED","intData":[10],"floatData":[12.3],"result":0.0,"text":""}
```

| Field     | Value                                                                                                                                                          |
|-----------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| id        | integer representing the recipient robot's ID                                                                                                                  |
| command   | string representing the command to be executed by the robot (see `ca.mcgill.ecse458.tcpserver.message.Command` for possible values)                            |
| status    | string representing the status of the command (all commands received by robots will bear the `DISPATCHED` status)                                              |
| intData   | integer array containing command parameters (can be empty, will be parsed by the robot code)                                                                   |
| floatData | float array containing command parameters (can be empty, will be parsed by the robot code)                                                                     |
| result    | double representing the result of the command (actual distance travelled, angle turned, etc -  all commands received will have a value of `0.0` in this field) |
| text      | string representing a textual message that the robot may include in a command response message (all commands received will not have any message - i.e. "")     |

After the robot executes the command (whether successfully or unsuccessfully), it is expected to send a response to the
server in the same format, with the following changes:

| Field     | Value                                                                                                      |
|-----------|------------------------------------------------------------------------------------------------------------|
| id        | unchanged                                                                                                  |
| command   | unchanged                                                                                                  |
| status    | `SUCCESS` if the command was fully completed, `FAILURE` if the command was not or only partially completed |
| intData   | unchanged                                                                                                  |
| floatData | unchanged                                                                                                  |
| result    | actual distance travelled, angle turned, etc                                                               |
| text      | any textual message that the robot wants to send to the server                                             |

Sample robot command and response messages may be found in the appendix below.

### META AI

Similarly to the robot API, commands from the META AI are expected as JSON-formatted strings in the following format:

```json
{"id":1,"command":"TURNRIGHT","status":"DISPATCHED","intData":[10],"floatData":[12.3],"result":0.0,"text":""}
```

| Field     | Value                                                        |
|-----------|--------------------------------------------------------------|
| id        | integer representing the recipient robot's ID                |
| command   | string representing the command to be executed by the robot  |
| status    | does not matter, but `DISPATCHED` should be used for clarity |
| intData   | integer array containing command parameters                  |
| floatData | float array containing command parameters                    |
| result    | does not matter, but `0.0` should be used for clarity        |
| text      | empty string                                                 |

After the command is processed, sent to the robot and executed, the robot response will be sent to the META AI in the
same format, with the following changes:

| Field     | Value                                                                                                      |
|-----------|------------------------------------------------------------------------------------------------------------|
| id        | unchanged                                                                                                  |
| command   | unchanged                                                                                                  |
| status    | `SUCCESS` if the command was fully completed, `FAILURE` if the command was not or only partially completed |
| intData   | unchanged                                                                                                  |
| floatData | unchanged                                                                                                  |
| result    | actual distance travelled, angle turned, etc                                                               |
| text      | any textual message that the robot wants to send to the server                                             |

For the get robot list command, a list containing the ID and type of each robot connected to the server is returned instead.

Note that the META AI may receive robot responses out of the order the commands were issued in.

Sample AI command requests and responses may be found in the appendix below.

### REST endpoint

Each command has its own HTTP endpoint that accepts its corresponding HTTP method and optionally (for some) a request body:

| Command         | Method | Endpoint   | Request body expected? |
|-----------------|--------|------------|------------------------|
| Move forward    | POST   | /forward   | Yes                    |
| Move backward   | POST   | /backward  | Yes                    |
| Turn right      | POST   | /turnright | Yes                    |
| Turn left       | POST   | /turnleft  | Yes                    |
| Ping            | POST   | /ping      | Yes                    |
| Get robot list  | GET    | /robots    | No                     |
| Change robot ID | POST   | /changeid  | Yes                    |

The request body is expected as a JSON-formatted string in the following format:

```json
{"id":1, "param":123.456}
```

| Field | Value                                                                                    |
|-------|------------------------------------------------------------------------------------------|
| id    | integer representing the recipient robot's ID                                            |
| param | double representing the parameter specific to the command (distance, angle, new ID, etc) |

After an HTTP request is made, the response body will contain a JSON-formatted string in the following format:

```json
{"id":1,"command":"TURNRIGHT","status":"DISPATCHED","intData":[10],"floatData":[12.3],"result":0.0,"text":""}
```

| Field     | Value                                                                    |
|-----------|--------------------------------------------------------------------------|
| id        | integer representing the recipient robot's ID                            |
| command   | string representing the command to be executed by the robot              |
| status    | `DISPATCHED` if the command and parameter are valid, `INVALID` otherwise |
| intData   | integer array containing command parameters                              |
| floatData | float array containing command parameters                                |
| result    | `0.0`                                                                    |
| text      | empty string                                                             |
This response is only meant as a confirmation of reception from the server and not as an actual robot response. To view
the robot response, please see the server console.

For the get robot list command, a list containing the ID and type of each robot connected to the server is returned in
the response body instead.

[Back to the top](#table-of-contents)


## Javadoc

This project's javadoc is accessible at `prometheus-swarm/TcpServer/docs/index.html`.

[Back to the top](#table-of-contents)


## Roadmap

- [x] Implement a dedicated TCP endpoint for the META AI
- [x] Add full Javadoc to classes
- [ ] Add validation for command parameters (e.g. no negative or zero params)
- [ ] Add support for extended naming (e.g. alphanumeric identifier in addition to the numeric robot ID)
- [ ] Implement acknowledgement response for robot when receiving a command (in addition to success/failure response upon completion)
- [ ] Implement periodic heartbeat message (poll each robot periodically to ensure health)
- [ ] Use protocol buffers as the communication medium between the AI, server and robots to enable platform-agnosticism
- [ ] More stuff?

[Back to the top](#table-of-contents)


## Appendix

### Sample API usage

#### Robot

Received move command from server:

```json
{"id":1,"command":"FORWARD","status":"DISPATCHED","intData":[],"floatData":[123.456],"result":0.0,"text":""}
```

Returning successful response to server:

```json
{"id":1,"command":"FORWARD","status":"SUCCESS","intData":[],"floatData":[123.456],"result":123.456,"text":""}
```

Returning failure response (`result` may also be `0.0`) with a text message (text may also be blank) to server:

```json
{"id":1,"command":"FORWARD","status":"FAILURE","intData":[],"floatData":[123.456],"result":100.456,"text":"who lives in a pineapple under the sea?"}
```

---

Received turn command from server:

```json
{"id":1,"command":"TURNRIGHT","status":"DISPATCHED","intData":[],"floatData":[123.456],"result":0.0,"text":""}
```

Returning successful response to server:

```json
{"id":1,"command":"TURNRIGHT","status":"SUCCESS","intData":[],"floatData":[123.456],"result":123.456,"text":""}
```

Returning failure response (`result` may also be `0.0`) with an error message (text may also be blank) to server:

```json
{"id":1,"command":"TURNRIGHT","status":"FAILURE","intData":[],"floatData":[123.456],"result":100.456,"text":"spongebob squarepants!"}
```

---

Received ping command from server:

```json
{"id":1,"command":"PING","status":"DISPATCHED","intData":[],"floatData":[],"result":0.0,"text":""}
```

Returning result to server:

```json
{"id":1,"command":"PING","status":"SUCCESS","intData":[],"floatData":[],"result":255.0,"text":""}
```

#### META AI

Sending move command to server:

```json
{"id":1,"command":"FORWARD","status":"DISPATCHED","intData":[],"floatData":[123.456],"result":0.0,"text":""}
```

Received successful response from server:

```json
{"id":1,"command":"FORWARD","status":"SUCCESS","intData":[],"floatData":[123.456],"result":123.456,"text":""}
```

Received failure response from server:

```json
{"id":1,"command":"FORWARD","status":"FAILURE","intData":[],"floatData":[123.456],"result":100.456,"text":"absorbent and yellow and porous is he!"}
```

---

Sending turn command to server:

```json
{"id":1,"command":"TURNRIGHT","status":"DISPATCHED","intData":[],"floatData":[123.456],"result":0.0,"text":""}
```

Received successful response from server:

```json
{"id":1,"command":"TURNRIGHT","status":"SUCCESS","intData":[],"floatData":[123.456],"result":123.456,"text":""}
```

Received failure response from server:

```json
{"id":1,"command":"TURNRIGHT","status":"FAILURE","intData":[],"floatData":[123.456],"result":100.456,"text":"spongebob squarepants!"}
```

---

Sending ping command to server:

```json
{"id":1,"command":"PING","status":"DISPATCHED","intData":[],"floatData":[],"result":0.0,"text":""}
```

Received response from server:

```json
{"id":1,"command":"PING","status":"SUCCESS","intData":[],"floatData":[],"result":255.0,"text":""}
```

---

Sending get robot list command to server:

```json
{"id":1,"command":"GETROBOTS","status":"DISPATCHED","intData":[],"floatData":[],"result":0.0,"text":""}
```

Received response from server:

```json
{"robots":[{"id":1,"robotType":"HOVERBOT"},{"id":2,"robotType":"BOEBOT"}]}
```

[Back to the top](#table-of-contents)


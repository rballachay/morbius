import json
import requests

endpoint = "http://localhost:8080/{}"
headers = {'content-type': 'application/json'}


def make_body(id, param):
    return {"id": id, "param": param}


if __name__ == '__main__':
    while True:
        command = int(input("""Select a command:
1 - Move forward
2 - Move backward
3 - Turn left
4 - Turn right
5 - Ping
6 - Change robot id
7 - List robots
"""))

        if command == 1:
            id = int(input("Enter the target robot's ID (integer): "))
            param = float(input("Enter a distance (double): "))
            response = requests.post(endpoint.format("forward"),
                                     data=json.dumps(make_body(id, param)), headers=headers)

            if response.status_code == 200:
                body = response.json()
                print("received response: ID: {} - Command: {} - Status: {} - Distance: {} \n"
                      .format(body['id'], body['command'], body['status'], body['distance']))
            else:
                print("got HTTP {}: {} \n".format(str(response.status_code), response.text))

        elif command == 2:
            id = int(input("Enter the target robot's ID (integer): "))
            param = float(input("Enter a distance (double): "))
            response = requests.post(endpoint.format("backward"),
                                     data=json.dumps(make_body(id, param)), headers=headers)

            if response.status_code == 200:
                body = response.json()
                print("received response: ID: {} - Command: {} - Status: {} - Distance: {} \n"
                      .format(body['id'], body['command'], body['status'], body['distance']))
            else:
                print("got HTTP {}: {} \n".format(str(response.status_code), response.text))

        elif command == 3:
            id = int(input("Enter the target robot's ID (integer): "))
            param = float(input("Enter an angle (double): "))
            response = requests.post(endpoint.format("turnleft"),
                                     data=json.dumps(make_body(id, param)), headers=headers)

            if response.status_code == 200:
                body = response.json()
                print("received response: ID: {} - Command: {} - Status: {} - Angle: {} \n"
                      .format(body['id'], body['command'], body['status'], body['angle']))
            else:
                print("got HTTP {}: {} \n".format(str(response.status_code), response.text))

        elif command == 4:
            id = int(input("Enter the target robot's ID (integer): "))
            param = float(input("Enter an angle (double): "))
            response = requests.post(endpoint.format("turnright"),
                                     data=json.dumps(make_body(id, param)), headers=headers)

            if response.status_code == 200:
                body = response.json()
                print("received response: ID: {} - Command: {} - Status: {} - Angle: {} \n"
                      .format(body['id'], body['command'], body['status'], body['angle']))
            else:
                print("got HTTP {}: {} \n".format(str(response.status_code), response.text))

        elif command == 5:
            id = int(input("Enter the target robot's ID (integer): "))
            response = requests.post(endpoint.format("ping"),
                                     data=json.dumps(make_body(id, 0.0)), headers=headers)

            if response.status_code == 200:
                body = response.json()
                print("received response: ID: {} - Command: {} - Status: {} \n"
                      .format(body['id'], body['command'], body['status']))
            else:
                print("got HTTP {}: {} \n".format(str(response.status_code), response.text))

        elif command == 6:
            id = int(input("Enter the target robot's ID (integer): "))
            param = float(input("Enter a new ID (integer): "))
            response = requests.post(endpoint.format("changeid"),
                                     data=json.dumps(make_body(id, param)), headers=headers)

            if response.status_code == 200:
                body = response.json()
                print("received response: ID: {} - Command: {} - Status: {} \n"
                      .format(body['id'], body['command'], body['status']))
            else:
                print("got HTTP {}: {} \n".format(str(response.status_code), response.text))

        elif command == 7:
            response = requests.get(endpoint.format("robots"))

            for robot in response.json()['robots']:
                print("ID: {} - Type: {}".format(robot['id'], robot['robotType']))

            print("")

        else:
            print("Please select a number corresponding to a command")

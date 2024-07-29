# Rasa Actions

The actions endpoint is automatically launched from the controller.py in the following lines:


```python
action_cmd = f"rasa run actions --actions {action_path} -p {actions_port}"

with open(rasa_logs, "a") as log_file:
    ...

    action_process = subprocess.Popen(
        action_cmd, shell=True, stdout=subprocess.PIPE, stderr=log_file
    )
    self.print(f"Rasa action server launched with PID: {action_process.pid}")
```

rasa is installed using `pip install rasa` and this makes a command-line tool accessible. We then launch a socket-based service on the port `actions_port` (defaults to 5055, from config.py). We don't need to handle any calls to this, as the endpoint at which the rasa model reaches the actions server is specified inside of the `endpoints.yml` file. If you ever want to change the port at which the actions server is called, just change that number in both of those places, `config.py` and `endpoints.yml`.

The actions are run as a secondary API that is called from the rasa model server. Each actions `name(self)` method must correspond to the name of that action in both the `domain.yml` file under the `actions:` header, and in the corresponding rule that will result in this action being called, inside of `data/rules.yml`. The extracted slots from the utterance that triggered the action can be accessed as follows (taken from `ActionMoveForward`):

```python
distance=None
for entity in tracker.latest_message['entities']:
    if entity['entity'] == 'distance':
        distance = entity['value']
```
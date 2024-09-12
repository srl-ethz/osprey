
## Success [0XX]

- Successful execution: 0
- (Feedback on execution...)
- Successful cancellation by request: 30;

## Bad Execution [1XX]

- General unsuccessfull execution: 100
- Timeout: 104


## Bad Argument / State [2XX]

- Argument not feasible: 200
- Request not feasible: 201

## MavSDK Error [3XX]

- MavSDK Error: 300 + <mavsdk_return_code>

## ROS2 Error [4XX]

- General ROS2 Error: 400
- Server not found: 404
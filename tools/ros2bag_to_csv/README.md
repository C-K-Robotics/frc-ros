# Convert ROS2 Bag to CSV - V0.2

## What's Required

- Python3
- sqlite3 if using on a DB3 file
- [mcap](https://pypi.org/project/mcap/) if using on a MCAP file
- Access to the definition of the message you want to convert (Not needed if you are using MCAP)

You do not need access to all the message definitions.

## What's Convertable Without Custom Message Handler

All message fields of type `int`, `float` and `str`. Nested messages needs to have its own handler.

## Use

python3 ros2bag_to_csv.py -i path/to/bag -t /topic/name -o output.csv

## Write your own handler

This example walks you through creating custom message handler.

You can add your own handler script to `handlers` folder

1. Create a handler file `handle_vector3.py` under `handlers`
2. Implement `get_what_i_can_handle()` which prompts the user what type of message it handles.
```Python
def get_what_i_can_handle():
    return "geometry_msgs/msg/Vector3"
```
3. Implement `get_handle_able_type()` which returns the message type it handles.
```Python
def get_handle_able_type():
    from geometry_msgs.msg import Vector3
    return Vector3
```
4. Implement `handle(msg)` which returns a dictionary of `field_name: value` to be outputed to the CSV file.
```Python
def handle(msg):
    return {
        "x": msg.x,
        "y": msg.y,
        "z": msg.z
    }
```

Alternatively, implement the functions in your own script elsewhere, and register your handler function.

```python
from handlers import register_handler
register_handler(get_what_i_can_handle(), 
                 get_handle_able_type(), handle)
```

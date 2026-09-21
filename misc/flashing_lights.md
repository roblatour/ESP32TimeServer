# RGB Status Light

When the optional RGB status light is installed and enabled, it provides a
quick indication of the time server's state.

| Light           | Meaning                                                             |
| --------------- | ------------------------------------------------------------------- |
| Blue            | The server is in startup mode.                                      |
| Flashing blue   | The server is almost ready, but needs to complete PPS disciplining. |
| Green           | The server is operating normally.                                   |
| White           | The server is synchronizing its time.                               |
| Flashing yellow | The MQTT service needs attention or has queued messages.            |
| Yellow          | A timing, pulse, or GNSS condition needs attention.                 |
| Flashing red    | The server cannot communicate with its GNSS receiver.               |
| Red             | Ethernet is not connected.                                          |

Flashing lights alternate between on and off once per second. When a condition
clears, the light changes to show the next applicable condition.
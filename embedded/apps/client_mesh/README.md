#### Bluetooth: Mesh Generic OnOff, Generic Level, Lighting & Vendor Models

##### Overview
********

This is a application demonstrating a Bluetooth mesh node in
which Root element has following models

- Generic OnOff Server
- Generic OnOff Client
- Vendor Model

Prior to provisioning, an unprovisioned beacon is broadcast that contains
a unique UUID. Each button controls the state of its
corresponding LED and does not initiate any mesh activity

##### Associations of Models with hardware
************************************

For the dwm1001-dev board, these are the model associations:

* LED1 is associated with generic OnOff Server's state which is part of Root element
* Button1 is associated with gen. OnOff Client which is part of Root element

States of Servers are bounded as per Bluetooth SIG Mesh Model Specification v1.0

After provisioning, the button clients must
be configured to publish and the LED servers to subscribe.
If a server is provided with a publish address, it will
also publish its relevant status.

##### Requirements
************
Part of this sample (persistent mesh config settings) has been tested on the dwm1001-dev board.


##### Running
************

Provisioning is done using the BlueZ meshctl utility. In this example, we'll use meshctl commands to bind:

- Button1 and LED1 to application key 1. It then configures Button1 publish to group 0xC000 and LED1 to subscribe to that group.

```
discover-unprovisioned on
provision <discovered UUID>
menu config
target 0100
appkey-add 1
bind 0 1 1000
bind 0 1 1001
sub-add 0100 c000 1000
pub-set 0100 c000 1 0 5 1001
```

The meshctl utility maintains a persistent JSON database containing
the mesh configuration. As additional nodes (boards) are provisioned, it
assigns sequential unicast addresses based on the number of elements
supported by the node. This example supports 2 elements per node.

The meshctl target for configuration must be the root element's unicast
address as it is the only one that has a configuration server model. If
meshctl is gracefully exited, it can be restarted and reconnected to
network 0x0.

The meshctl utility also supports a onoff model client that can be used to
change the state of any LED that is bound to application key 0x1.
This is done by setting the target to the unicast address of the element
that has that LED's model and issuing the onoff command.
Group addresses are not supported.


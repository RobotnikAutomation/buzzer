# buzzer package

Shows a service to activate the buzzer with a frequency.

## buzzer_node

## Parameters

* ~service_io (string, default: "set_digital_output")
  Name of service that the node can use for activate the buzzer

* ~digital_output (int, default: 1)
  Number of the digital output where the buzzer is connected
  
## Published Topics

* ~state (robotnik_msgs/State)
  State of the node
  
## Services

* set_buzzer (buzzer/SetBuzzer)
  Set the state of the buzzer from a client
  
## Services Called
* set_digital_output (robotnik_msgs/set_digital_output)
  Sends the desired state of the buzzer to the hw

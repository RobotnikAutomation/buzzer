# buzzer package

Shows a service to activate the buzzer with a frequency.

## buzzer_node

## Parameters

* ~topic_io (string, default: "set_digital_output")
  Topic that the node can use for activate the buzzer

* ~digital_output (int, default: 1)
  Digital output when the buzzer is connected
  
## Published Topics

* ~state (robotnik_msgs/State)
  State of the node
  
## Services

* set_buzzer (buzzer/SetBuzzer)
  Set the state of the buzzer by client
  
## Services Called
* set_digital_output (robotnik_msgs/set_digital_output)
  Set the state of the buzzer to the hw

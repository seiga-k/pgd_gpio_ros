# pgd_gpio_ros

ROS node for handling GPIO on Raspberry PI with pigpio. 

# Dependencies

This package use pigpiod to handle a GPIO on Raspberry PI.
Please install and run pigpio first.

# param

Recomended to use yaml for parameter configuration.

Example is as bellow.

```yaml
PigpiodIP: "172.20.0.97"
PigpiodPort: "8888"
input:
    port:   [16,    20,     21]
    invert: [false, true,   false]
    pull:   [down,  up,     off]
    event:  [true,  true,   false]
    filter: [0,     0,      10]
output:
    port:       [5,     6]
    invert:     [false, true]
    default:    [0,     1]
```

- PigpiodIP : IP address of Raspberry PI runs pigpiod.
- PigpiodPort : Portnumber of pigpiod on Raspberry PI.
- input : input element has five sub-element. Each elements are array type.
    - port : Array of the port number for input use.
    - invert : Array of ther port inverting configuration. When GPIO hardware is high, the output will publish zero if set true for an invert.
    - pull : Array of the port pull-up / pull-down setting. The element value must be choosee from "down", "up" and "off".
    - event : Array of the event trigger configuration. If set true, pin level change make publish.
    - filter : Array of the glitch filter configuratin. Set integer value in micro second.
- output : input element has three sub-element. Each elements are array type.
    - port : Array of the port number for output use.
    - invert : same as input.
    - default : Array of the port value for the default.

Input and Output parameters are configured as the order of the array.

# message

## gpio

Please reffer a msg file.

# topic

## subscribe

- ~output : msg type pgd_gpio_ros/gpio

## publish

- ~input : msg type pgd_gpio_ros/gpio
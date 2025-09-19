servomotor_command --help   # get help information about this command line utility
servomotor_command -c       # print out all commands currently available
servomotor_command -P       # select the serial port to be used for communication from a menu
servomotor_command "Detect devices"  # detect connected servomotors

# The following will set the alias for a device. Use the unique ID discovered from the
# "Detect devices" command. The example sets the alias to X. After the alias is set, you
# can send commands to the device and target it with the alias instead of the unique ID. 
servomotor_command -a 1122334455667788 "Set device alias" X  # set the alias for a device

servomotor_command -a X "Enable mosfets" # enable the MOSFETS of the servomotor with alias X
servomotor_command -p /dev/ttyUSB0 "Enable mosfets"  # same as above but hard specify the port
servomotor_command -a X "Trapezoid move" 3000000 32000  # move the motor
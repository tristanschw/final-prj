# Group_work

This package contains the code that Aryaman and Suphakorn wrote to control
and interface with the hand.

## Breakdown of main files

Main.Launch - Launch file for the hand controller(s)

Master.Launch - Launch file that integrates all the nodes together.

allegro_node_torque.cpp/.h - Combined torque and position controller. Cut these and overwrite them into the Allegro_hand library. 

close_to_open.py - Opens the chopsticks

open_to_close.py - Closes the chopsticks

my_robot.xacro - Xacro file to generate the combined URDF file.

All other files were not fully implemented, or were used for testing purposes. 




## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)

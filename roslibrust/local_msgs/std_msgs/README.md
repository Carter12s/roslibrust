# Local std_msgs Copy

This was copied from ros/std_msgs kinetic-devel branch on 4/11/22

We could have done a git sub-module and might switch to that in the future, but since these seem very static
at this point, I decided to just static copy them.


NOTE: 
This folder contains "undefined" custom types that are special cases in the ros message definition.

These could be hard coded into the lib with custom logic, but is simple this way for now.

This folder is automatically included as an additional search path

NOTE: there is a pre-existing std_msgs::Time which is why the 'I' suffix is added to names

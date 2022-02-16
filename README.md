# Build interface
## Basic introduction
This repo serves for setting up the interface between the kuka robot and data processor (PC-end). 
In general, it consists of following work: 
- UDP networking between Kuka robot and PC that processes the packet sent from Kuka robot is established. During processing,
the packet containing the xml data would be parsed so that desired data is extracted and sent back to robot.


## Toolchain
### Windows
> Toolset: Microsoft Visual Studio 2019
> 
> Build systems: CMake (version: 3.21.1)
> 
> Make: mingw32-make.exe
> 
> Compilers: gcc and g++ (working version: 9.3.0) or GNU (working version: 11.2.0) ????????
> 
> Debugger: LLDB
### Ubuntu (POSIX platform)
> Toolset: 
>
> Build systems: CMake (version: 3.21.1)
>
> Make: mingw32-make.exe
>
> Compilers: gcc and g++ (working version: 9.3.0) or GNU (working version: 11.2.0) ????????
>
> Debugger: LLDB


## Essential libraries
1. Boost
> - Boost with version "1.73.0" (tested) or "1.75.0" (tested)
2. XML parsing
> - XercesC with version "3.2.3" (**Note**:To get the code to run in order, please use XercesC with version greater than 3.x)
3. ROS
> - ROS Noetic Ninjemys


## Usage
Pull from git repo:
1. `git clone https://github.com/lenardxu/kuka_interface.git`
2. Open this repo as a project with CLion; Configure the CMake and Toolchains of CLion so that the toolchain for building 
this project is **Visual Studio** since the dependencies - Boost and Xerces C++ - are built using **Visual Studio**.
3. `cd kuka_interface`
4. `mkdir build`
5. `cd build`
6. `cmake ..` 
    
    or `cmake -G "Visual Studio 16 2019" ..` when visual studio is to be selected as compiler

    or `cmake -G "Visual Studio 16 2019" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<install path> ..` when the 
    program should be released and the install dir should be specified. Then don't forget to navigate to the dir of `<install path>`
7. `cmake --build .`
8. `./build_interface_server run` in one console under *./build/* directory when running on a POSIX platform;
`.\build_interface_server run` in one console under *.\build\Debug* directory when running on a WINDOWS platform
- **run** means non-debugging mode, while **debug** means debugging mode activated.
9. `./build_interface_client` in one console under *build/* directory when running on a POSIX platform;
`.\build_interface_client` in one console under *.\build\Debug* directory when running on a WINDOWS platform 
(**Note**: this client is a simulated robot under testing.)

### Build and run package(s) based on ROS1 on Windows
- General process of build zero to many catkin packages in a workspace:
  - Command `catkin_make` under your workspace in a terminal to build any catkin projects found in the "src" folder or 
  `catkin_make --source my_src` to build them found based on source code in another place e.g. named as "my_src" 
  - Command `catkin_make install` next or `catkin_make install --source my_src`
- Source your new setup.bat file under the same workspace using: `devel\setup.bat` (Windows) instead of 
`source devel/setup.bash` (Linux)
- Open a new terminal and command `roscore` in order for ROS nodes to communicate. [Helpful link](http://wiki.ros.org/roscore)
- Return to the previous terminal, and command `rosrun process_msg_ki generate_robot_msg_test` under the same workspace
- Open another new terminal, and command `rosrun process_msg_ki process_robot_msg_node` under the same workspace
- Open another new terminal, and command `rostopic list` to find the running `topic` specified by you in code, and then 
command `rostopic echo <your_topic>`

## Q&A
> Why can't the compiler like Visual studio or gcc and c++ managed by MinGW find the "sys/socket.h" and other related 
> header files?
>>   Answer: Such header files are specific to POSIX platform, which are not supported by WINDOWS. As an alternative for
networking programming on Windows, which is probably the only one, you may refer to WinSock (sockets api on windows), 
which supports UDP ad TCP networking.

## Local network setup (for simulation of connection between robot and computer)
In the mode of two-computer-ethernet-connection,
1. Connect two computers with one router with two internet cabels respectively
   1. Check whether the ethernet connection of Windows computer is activated like [this image](./docs/ethernet_desired_state_on_windows.PNG)
   shows, that the **ethernet** (main) other than e.g. **ethernet 3** is activated
2. Set ip addresses for each computer respectively, e.g. "192.168.1.1" for PC 1 and "192.168.1.2" for PC 2; And set the 
same subnet mask for each computer, typically "255.255.255.0"
   1. **Note**: It is necessary to keep the network segments for both ip addresses the same - the example ip addresses and 
   subnet masks can achieve that
3. Command `ping 192.168.1.2` on PC1's terminal and command `ping 192.168.1.1` on PC2's terminal to check the connection
between these two computers under the ethernet
   1. If there is the loss of 0%, it indicates the success; Otherwise, it fails. To troubleshoot it, you can try commanding
   `ipconfig` to see the ipv4 address if it is the same as you set
   2. Or check if it is related with internet firewall - try closing internet firewall when you want to connect these two
   computers
4. After successful check, it is then able to connect two computers using given code of udp network programs in socket. To
be noted, the `server.sin_addr.s_addr = inet_addr("0.0.0.0")` or `server.sin_addr.s_addr = htonl(INADDR_ANY)`should be 
applied other than `server.sin_addr.s_addr = INADDR_ANY` for the server ip address. The reason is that **sockaddr_in** requires
value of ip address to be in network-byte order, while **INADDR_ANY** is an unsigned long int in host byte order. As a 
result, this constant needs to be explicitly converted by using the function **htonl**, which deals with 32-bit values 
(**htons** handles 16-bit values).

## Others
- Instead of manually making a build directory in th current directory, commanding `cmake -Bbuild -G "Visual Studio 16 2019" ..`
is also an option for this purpose.

### About Shared Memory

### Clion and ROS
- Please refer to [this link](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html) to check how to setup ROS
for Clion.
- When editing the path value for any environment variable in CMakeLists.txt, please adere to UNIX or LINUX typing style
instead of Windows style even if you are developing under Windows OS. Example: "D:/SUMOLibraries-main/xerces-c-3.2.3/src"
instead of "D:\\SUMOLibraries-main\\xerces-c-3.2.3\\src", otherwise you would encounter path parsing problem.

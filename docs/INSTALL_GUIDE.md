# Installation guide for windows

1. Install Clion
2. Install gcc and g++ (Not necessary when you would use MCVS2019)
   1. Prerequisites: C++ compiler on Windows (Microsoft Visual Studio 2019) already installed
      1. Install Microsoft Visual Studio 2019 with [reference link](https://docs.microsoft.com/en-us/visualstudio/install/install-visual-studio?view=vs-2022)
      2. Upgrade Microsoft Visual Studio 2019 if any by searching and launching **Visual Studio Installer**
   2. To install these two compilers, MSYS2 (instead of MinGW itself, due to version limitation till 6.3.0 w.r.t. GCC) 
   software distribution is required on Windows. 
   Please download it from [the official page](https://www.msys2.org/).
   A useful reference link for installation of gcc and g++ is [here](https://solarianprogrammer.com/2019/11/05/install-gcc-windows/).
   Another useful video reference for installation of gcc and g++ is [here](https://www.youtube.com/watch?v=7z1QkzdRcoY&list=PL1_C6uWTeBDEzpCjYI7Seq6yh8ZrlCC8C).
      1. Note: when the upgrade of gcc and g++ is a necessity, please also refer to [the related video link](https://www.youtube.com/watch?v=JlDssVh5Kd0)
      2. Note: when the older version of gcc and g++ is a necessity, please refer to [this link from stack overflow](https://stackoverflow.com/questions/33969803/how-to-obtain-older-versions-of-packages-using-msys2)
   3. To check the installation, you can command `gcc -v` and `g++ -v` under the opened exe named **MSYS2 MinGW x64**
   4. Set environment path
      1. Search in Windows "environment"
      2. Add new variable `MSYS2_DIR` with value e.g. `D:\msys64` (i.e. msys2 main dir after installation) to system variables
      3. Add new path `%MSYS2_DIR%\mingw64\bin` to **Path** of user variable
      4. Add new path `%MSYS2_DIR%\usr\bin` to **Path** of user variable
      5. Check the installation by commanding `gcc -v` and `g++ -v` under any command prompt or powershell
3. Install boost libraries on Windows (Not necessary when you would use ROS since it is built in.)
   1. Using Microsoft Visual Studio 2019 for compiling
      1. [Reference link](https://www.youtube.com/watch?v=5afpq2TkOHc)
      2. General steps:
         1. Download the boost zip file (version: 1.73.0 from the [link](https://www.boost.org/users/history/version_1_73_0.html)
         or version: 1.75.0 from the [link](https://www.boost.org/users/history/version_1_75_0.html))
         2. Extract the zip file to "C:\DevSoft\boost" -> Folder named "boost_1_75_0"
         3. Navigate to "boost_1_75_0" in powershell and then check the directory 
         4. Command `bootstrap vc142` or `.\bootstrap vc142` (possibly required on Windows) to prepare b2
         5. Command `.\b2` as recommended to build boost libraries
         6. [Optional] Add new variable `INCLUDE` with value `C:\DevSoft\boost\boost_1_75_0` to user varaibles
         7. [Optional] Add new variable `LIB` with value `C:\DevSoft\boost\boost_1_75_0\stage\lib` to user varaibles
         8. Since the generated c++ boost libraries (static due to .lib) are relatively small, no need for building 
         dynamic link libraries
   2. **NOTE**: 
      1. If some source files or header files are indicated missing when using VS2019 to build boost, it can be possbily
      due to the partial loss of current VS2019. It is then recommended to reinstall **Desktop development with C++** suite
      of VS2019 through its built-in **Visual Studio Installer**.
      2. The toolset version is v140 for Visual Studio 2015, v141 for 2017, and v142 for 2019
      3. If you would still build dll after building static libraries, please still refer to the video reference in item 1.
   3. Using gcc for compiling
      1. [Reference link](https://gist.github.com/zrsmithson/0b72e0cb58d0cb946fc48b5c88511da8). This way may cost much more 
      time to build boost libraries according to the user experience.
4. Install Xerces-C++ (version: 3.2.3) - **Variant 1** (Optional)
   1. Before conducting any related with Xerces-C++, 
      1. [For building our project] First include cmake bin dir to system path by adding value e.g. 
      `D:\JetBrains\CLion 2021.3.2\bin\cmake\win\bin` to `Path` of user variables if you've used Clion as your IDE, since cmake
      is already integrated into Clion. Otherwise, you would have to manually download 
      ([downloading source](https://cmake.org/files/) for zip files) and build cmake with some compiler and include its
      bin directory (e.g., `...\cmake-3.21.1-rc3-win64-x64\bin`) to `Path` of user variables as above.
      2. Then install and build **mingw version of cmake** for building Xerces-C++, under **MSYS2 MinGW x64** console
         1. Reference [link](https://www.msys2.org/docs/cmake/) for installation and building of **mingw version of cmake**
         2. command `pacman -S mingw-w64-x86_64-cmake`
         3. [optional]command `pacman -S mingw-w64-x86_64-ninja` is not necessary when **Visual Studio** is used
      3. Then run powershell as administrator, and command `mklink /J D:\MinGW "D:\msys64\mingw64"` to create a link to 
      MinGW folder in D:\ so that any changes in this "link" folder would not have an impact on the original if you remove
      it later on.
   2. Download the Xerces-C++ zip file (version: 3.2.3) from the [link](https://xerces.apache.org/xerces-c/download.cgi)
   and extract it to **D:\MinGW**.
   3. Build Xerces-C++ libraries by referring to [this link](https://xerces.apache.org/xerces-c/build-3.html)
      1. General steps (running **MSYS2 MinGW x64**):
         1. `mkdir build` under Xerces-C++ project dir, which should be **D:\MinGW\xerces-c-3.2.3**
         2. `cd build`
         3. `cmake -G "Visual Studio 16 2019" ..` or `cmake -G "Visual Studio 16 2019" -DCMAKE_INSTALL_PREFIX=/mingw64/xerces-c-3.2.1/libs ..` when install dir is to be self-defined
         4. `cmake --build . --config Debug`
         5. `ctest -V -C Debug -j 4`
         6. `cmake --build . --config Debug --target install`
5. Install Xerces-C++ (version: 3.2.3) - **Variant 2 [simpler]** (Optional)
   1. Download zip file from the [repo](https://github.com/DLR-TS/SUMOLibraries)
   2. Extract **xerces-c-3.2.3 (already built and installed)** to some place, e.g., **D:\\SUMOLibraries-main\\xerces-c-3.2.3\\**
   3. Edit the corresponding variables specific to *CMakeLists.txt* of our project
   4. After building our project, you may need to copy all *.exes and *.dlls to the directory where the generated *.exes
   of our project are located, normally, **.\build\Debug**, so that the dlls can be correctly found.

## Install ROS1
> Ref.: http://wiki.ros.org/Installation/Windows It included the guide for install, upgrade and uninstall.
- **Powershell**, not **CMD**, should be checked, whose exe should normally be installed in `C:\Windows\System32\WindowsPowerShell\v1.0`
on 64-bit computer. Note: On a 64-bit computer, 64-bit programs store their files in `C:\Program Files`, and the system-wide 
`C:\Windows\System32` folder contains 64-bit libraries.
- To exclude the required directories from virus scanning as stated in the instruction, you can refer to 
[this link](https://www.windowscentral.com/how-exclude-files-and-folders-windows-defender-antivirus-scans)
- If there are errors bundled with invoking wrong path to python exe taking place when using `catkin_make`, you should remove that path from the 
System variable `Path` to avoid that conflict when you are using Windows. For example, when you've installed anconda, then
conda Python will be included in the System variable `Path`, so you need to remove all related system variables so that
only system Python will be invoked. A useful [reference](https://conda.io/projects/conda/en/latest/user-guide/troubleshooting.html#programs-fail-due-to-invoking-conda-python-instead-of-system-python) can be leveraged.
# Others
- When it is necessary to run Visual Studio as an administrator, please refer to [this link](https://docs.microsoft.com/en-us/visualstudio/ide/user-permissions-and-visual-studio?view=vs-2019#run-visual-studio-as-an-administrator) for how-to.
ViennaMini
--------------------------

Developer repository for ViennaMini, a C++-based classical semiconductor device simulator.
ViennaMini is currently in a prototype state.

System requirements
--------------------------

* C++ compiler
* CMake >= 2.8.2
* UDUNITS >= 2.1.23 (optional: for automatic unit conversions)

Currently supported operating systems
--------------------------
* GNU/Linux

Building instructions
--------------------------

To build ViennaMini, clone the repository and issue the following suggested commands (the following steps are for Unix-based systems):

<pre>
$> cd viennamini-dev  # the checked-out GIT folder
$> mkdir build        # the build folder
</pre>

External Vienna* dependencies can either be automatically downloaded (see 'DOWNLOAD' CMake option below)
or the required projects can be downloaded manually and pointed to by environment variables,
the latter being more suitable for repeatedly builds.
The following provides a list of required projects, their git URLs, the required git branches, and the required environment variables.
If you provide these, you can avoid using the DOWNLOAD option, thus rendering a redownload each new build-from-scratch obsolete.

<pre>
ViennaFVM       - https://github.com/viennafvm/viennafvm-dev.git              - master  - VIENNAFVMPATH
ViennaGrid      - https://github.com/viennagrid/viennagrid-dev.git            - next    - VIENNAGRIDPATH
ViennaData      - https://github.com/viennadata/viennadata-dev.git            - master  - VIENNADATAPATH
ViennaMath      - https://github.com/viennamath/viennamath-dev.git            - master  - VIENNAMATHPATH
ViennaCL        - https://github.com/viennacl/viennacl-dev.git                - master  - VIENNACLPATH
ViennaMaterials - https://github.com/viennamaterials/viennamaterials-dev.git  - next    - VIENNAMATERIALSPATH
ViennaMesh      - https://github.com/viennamesh/viennamesh-dev.git            - next    - VIENNAMESHPATH
</pre>

Configure the build, default build type is the 'optimized/release' mode:
<pre>
$> cd build/
$> cmake ..
</pre>

Now build and install the executable and libraries (install folder is a subfolder of the build folder):
<pre>
$> make -j4  # adjust to your CPU core count for efficient parallel building
$> make install
</pre>

CMake Options
--------------------------

<pre>
VERBOSE            =(ON), OFF         # Make ViennaMini output debug messages (default: ON)
DOWNLOAD           = ON, (OFF)        # Automatically checkout external dependencies during the build-process, i.e., other Vienna* projects (default: OFF)
BUILD_EXAMPLES     = ON, (OFF)        # Build the examples. The ViennaMini library is generated anyway (default: OFF)
USE_UNITS          =(ON), OFF         # Manually switch on/off units support (default: ON)
CMAKE_BUILD_TYPE   = debug, (release) # Turn off/on optimizations (default: release, i.e., optimized mode)
BUILD_SHARED_LIBS  =(ON), OFF         # Build all libraries as shared libraries, if switched off, static libraries are generated and used (default: ON)
</pre>

Automatic Unit Conversions
--------------------------

For automatic unit conversions of input quantities provided by the user, such as doping concentrations (1/cm3 -> 1/m3),
the UDUNITS library (http://www.unidata.ucar.edu/software/udunits/) has to be available on the target platform.
The library is available on major linux distributions, like Linux Mint, via the distribution's package installer.
However, the UDUNITS library is not required. If the library is not available on the system and incompatible units
are detected by ViennaMini, the simulation is aborted and an appropriate error message is raised.


Authors and Contact
------------------------

Josef Weinbub, Andreas Morhammer, Karl Rupp
(viennastar AT iue DOT tuwien DOT ac DOT at)

ViennaMini was developed under the aegis of the 'Institute for Microelectronics' and the 'Institute for Analysis and Scientific Computing' at the 'Vienna University of Technology'.


License
--------------------------
ViennaMini is distributed under the MIT (X11) License. See file LICENSE.

ViennaMini
--------------------------

Developer repository for ViennaMini, a C++-based classical semiconductor device simulator.
ViennaMini is currently in a prototype state. 

System requirements
--------------------------

* C++ compiler
* CMake >= 2.8.2

Currently supported operating systems
--------------------------
* Linux

Building instructions 
--------------------------

To build ViennaMini, simply clone the repository and issue the following suggested commands (the following steps are for Unix-based systems):

<pre>
$> cd viennamini-dev  # the checked-out GIT folder 
$> mkdir build             # the build folder
</pre>

Configure the build, default build type is the 'optimized/release' mode:
<pre>
$> cmake ..
</pre>

Now build and install the executable and libraries (install folder is a subfolder of the build folder):
<pre>
$> make -j4  # adjust to your CPU core count for efficient parallel building
$> make install
</pre>

4. Authors and Contact
------------------------

Karl Rupp (rupp@iue.tuwien.ac.at)
Josef Weinbub (weinbub@iue.tuwien.ac.at)

ViennaMini was developed under the aegis of the 'Institute for Microelectronics' and the 'Institute for Analysis and Scientific Computing' at the 'Vienna University of Technology'.


License
--------------------------
ViennaMini is distributed under the MIT (X11) License. See file LICENSE.


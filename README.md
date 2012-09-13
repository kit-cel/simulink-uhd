Simulink-UHD
============

The Simulink-UHD project is an Open Source Software Package that enables owners of UHD-compatible *Universal Software Radio Peripherals (USRP)* from *Ettus Research* to build models in MATLAB Simulink that interface with the device in real-time. Simulink-UHD uses the *USRP Hardware Driver (UHD)* library. It is thus possible to build a wide variety of signal processing and wireless communication applications directly in Simulink while being able to test them on real hardware at the same time.

The Simulink-UHD project was initiated at the *Communication Engineering Lab (CEL)* at the *Karlsruhe Institute of Technology (KIT)*, Germany, <http://www.cel.kit.edu>.

For more information, please visit the Simulink-UHD [project page](http://www.cel.kit.edu/simulink_uhd.php).


Requirements
------------

- MATLAB/Simulink (R2009b or newer) and *MEX* compatible [compiler](http://www.mathworks.de/support/compilers)

- *UHD* library from the [Ettus Research](http://code.ettus.com/redmine/ettus/projects/uhd/wiki "UHD homepage")

- *Boost C++* libraries from [here](http://http://www.boost.org/ "Boost homepage")

The Simulink-UHD software package doesn't come with pre-build binaries due to license issues, but the compilation is quite simple. 


Build/Install instructions for Microsoft Windows
------------------------------------------------

1. Get and build UHD or download the [pre-build binaries](http://code.ettus.com/redmine/ettus/projects/uhd/wiki/UHD_Windows). The latest successfully tested version is 003.004.003. If you are using a 64-bit MATLAB, make sure to get a 64-bit build of UHD.

2. After installing UHD, please make sure it is working. Plug-in and connect your *USRP device* and run one the tools that came with UHD, e.g.

		>uhd_find_devices.exe
		Win32; Microsoft Visual C++ version 10.0; Boost_104700; UHD_003.004.003-26-stable

		--------------------------------------------------
		-- UHD Device 0
		--------------------------------------------------
		Device Address:
			type: usrp2
			addr: 192.168.10.52
			name:
			serial:
		
	If you have not added UHD to your PATH, these tools are found the bin folder of your UHD installation (usually `C:\Program Files\UHD\bin`).

3. Download the Boost libraries from [Sourceforge](http://sourceforge.net/projects/boost/files/boost/ "Boost download page") and extract them to e.g. `C:\boost`.

4. Get the Simulink-UHD source from [GitHub](https://github.com/kit-cel/simulink-uhd).

		$ git clone git://github.com/kit-cel/simulink-uhd.git
		
	If you aren't using *git*, you can download a [compressed file](https://github.com/kit-cel/simulink-uhd/zipball/master) from the project page directly. Extract the archive to any folder you want.

5. Run MATLAB and setup the *MEX* compiler
	
		>> mex -setup
	
	(Successfully tested with Microsoft Visual Studio 10)

6. Switch to your Simulink-UHD directory.

7. Open and modify `config.m` to point to your *Boost* and *UHD* directory. 

		% UHD install directory
		UHD_DIRECTORY = '';
		% Boost include directory
		BOOST_DIRECTORY = '';
		% Boost library directory
		BOOST_LIB_DIRECTORY = '';

	Afterwards run `config.m` to generate the settings.mat file required for `make.m`

9. Start the build process 

		>> make

	For additional options run `help make`.

10. Add the directories `bin`, `blockset`, `help` and `utils` to your MATLAB path environment.

12. You will now find a new Toolbox named `Simulink-UHD` in the *Simulink Library Browser* (A restart of Simulink might be required for that). Additionally, a simple spectrum scope model is located in the directory `demo`.


Build/Install instructions for Linux
------------------------------------

The build instructions for Linux-based systems are a little more extensive/complicated than for Windows. The reason for that is the dependency for Boost in both the UHD library and MATLAB. As MATLAB uses its own Boost libraries (and therefore fiddles with the `LD_LIBRARY_PATH`), UHD has to be compiled with the same version of Boost as shipped with MATLAB. If you already have an existing UHD installation, check the versions:

- Locate one of the MATLAB Boost libraries and check the filename

		$ locate libboost_regex
		MATLAB_ROOT/bin/glnx*/libboost_regex.so.1.44.0

- Locate the libuhd library (usually /usr/lib) and check its dependencies

		$ ldd /usr/lib/libuhd.so.003.004 | grep boost
			  libboost_date_time.so.1.46.1 => /usr/lib/libboost_date_time.so.1.46.1 (0x00007f6520b16000)
			  libboost_filesystem.so.1.46.1 => /usr/lib/libboost_filesystem.so.1.46.1 (0x00007f65208f8000)
			  libboost_regex.so.1.46.1 => /usr/lib/libboost_regex.so.1.46.1 (0x00007f65205f5000)
			  libboost_system.so.1.46.1 => /usr/lib/libboost_system.so.1.46.1 (0x00007f65203f1000)
			  libboost_thread.so.1.46.1 => /usr/lib/libboost_thread.so.1.46.1 (0x00007f65201d8000)

If the versions of the Boost libraries differ as in the example above or you haven't an UHD installation yet, you need to build UHD with the correct version of Boost yourself. If the versions are the same, continue with step 6.

The following steps are exemplary for Ubuntu 12.04 Precise Pangolin, MATLAB R2012a and UHD 003.004.003:

1. Download the Boost libraries from [Sourceforge](http://sourceforge.net/projects/boost/files/boost/ "Boost download page"). It has to be the same version as MATLAB uses, here 1.44. Extract the archive, enter the Boost folder and start the configuration

		$ ./bootstrap

	The Boost folder will be called `BOOST_ROOT` below.

2. If no errors or missing dependencies occurred, start the Boost build process for the needed libraries

		$ ./bjam --layout=tagged --with-date_time --with-filesystem --with-program_options \
				--with-regex --with-system --with-thread --with-test threading=multi

	Since the Boost libraries are only needed for the Simulink-UHD software package, you shouldn't install them system-wide (No `sudo ./bjam install`).

3. Get the source code of the [UHD from Ettus Research](http://ettus-apps.sourcerepo.com/redmine/ettus/projects/uhd/wiki/UHD_Build ) and provide the prerequisites as explained. The UHD folder will be called `UHD_ROOT` below.

4. Follow the Build Guide until the point of generating the Makefiles with CMake. It is important to tell the UHD build system to use the self-build Boost libraries. For example, change the `Boost_DATE_TIME_LIBRARY` path to `BOOST_ROOT/stage/lib/libboost_date_time-mt.so`. We recommend to use the graphical CMake tools ccmake or cmake-gui for doing so. Both are available in the Ubuntu package manager.

5. Generate the Makefiles and start the UHD build process

		$ make

	Since the UHD library is only needed for the Simulink-UHD software package, you shouldn't install it system-wide (No sudo make install). After a successful build, check the dependencies of the self-build UHD library

		$ ldd UHD_ROOT/host/build/lib/libuhd.so.003.004 | grep boost
			  libboost_date_time-mt.so.1.44.0 => BOOST_ROOT/stage/lib/libboost_date_time-mt.so.1.44.0 (0x00007fca6bfc6000)
			  libboost_filesystem-mt.so.1.44.0 => BOOST_ROOT/stage/lib/libboost_filesystem-mt.so.1.44.0 (0x00007fca6bda2000)
			  libboost_regex-mt.so.1.44.0 => BOOST_ROOT/stage/lib/libboost_regex-mt.so.1.44.0 (0x00007fca6ba9c000)
			  libboost_system-mt.so.1.44.0 => BOOST_ROOT/stage/lib/libboost_system-mt.so.1.44.0 (0x00007fca6b898000)
			  libboost_thread-mt.so.1.44.0 => BOOST_ROOT/stage/lib/libboost_thread-mt.so.1.44.0 (0x00007fca6b67e000)

	If the Boost version matches MATLAB's Boost version, continue.

6. Get the *Simulink-UHD* source from [GitHub](https://github.com/kit-cel/simulink-uhd)

		$ git clone git://github.com/kit-cel/simulink-uhd.git

	If you aren't using git, you can download a compressed file from the project page directly. Extract the archive to any folder you want.

7. Run MATLAB and setup the MEX compiler

		>> mex -setup

	(Successfully tested with gcc 4.6.3)

8. Switch to your Simulink-UHD directory. Open and modify `config.m` to point to your Boost and UHD directory.

		% UHD install directory
		UHD_DIRECTORY = '';
		% Boost include directory
		BOOST_DIRECTORY = '';

	Afterwards run `config.m` to generate the settings.mat file required for `make.m`.

9. Start the build process

		>> make

	For additional options run help make.

10. Add the directories `bin`, `blockset`, `help` and `utils` to your MATLAB path environment.

11. You will now find a new Toolbox named *Simulink-UHD* in the *Simulink Library Browser* (A restart of Simulink might be required). Additionally, a simple spectrum scope model is located in the directory demo. 

12. **Important**: Since MATLAB uses the standard search-paths for external libraries, add the path of the self-build UHD library to the search-path

		$ export LD_LIBRARY_PATH=UHD_ROOT/host/build/lib
		$ matlab

	Alternatively, you can copy the `libuhd` to the MATLAB library folder `MATLAB_ROOT/bin/glnx*`.

13. Check if the Simulink-UHD software package is working properly

		>> uhd_find_devices
		linux; GNU C++ version 4.6.3; Boost_104400; UHD_003.004.003-177-g584b7ae2
		Found 1 device(s):
		Identifier    Type  Name
		==============================
		192.168.10.52 usrp2 

Changelog
---------

Version 1.0

- first release of the Simulink-UHD software package
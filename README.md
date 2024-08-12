									
# Mashup Drawings Demo 
by Greg Philbrick (Copyright 2024)
													
## Description	

This demonstrates my algorithm for "mashing up" two drawings, i.e., creating a "mashup
drawing" from two input drawings. Here, a "drawing" is a collection of strokes, each comprising
a path curve and a width curve (Core/model/stroke.h). To understand how the mashup process works,
imagine starting out with two input drawings superimposed on top of each other. The task is to 
create an output drawing that captures both input drawings, which sometimes involves smoothly 
joining Drawing A strokes with Drawing B strokes and sometimes involves having Drawing A strokes 
taper off and yield to Drawing B strokes (or vice versa). 

To see the mashup algorithm in action, run the MashupDemo executable. This goes through a few 
different scenarios. For each scenario, it outputs SCENARIONAME_inputs.eps and 
SCENARIONAME_mashup.eps (at the run-from location). These are Encapsulated PostScript files: 
vector graphics files that can be viewed in Illustrator or Inkscape. The former shows the two 
input drawings--Drawing A in red and Drawing B in blue. The latter shows the mashup drawing created
by the algorithm.

If you just want to scan the code for an understanding of the algorithm, go to Mashup/blendrawings.h 
and study the BlendDrawings constructor and BlendDrawings::perform().

## How to Build / 3rd-Party Dependencies

This project has four 3rd-party dependencies:

* [Clipper2](https://angusj.com/clipper2/Docs/Overview.htm)	
* [GeometricTools](https://www.geometrictools.com/)
* [Boost](https://www.boost.org/)		
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
	
I have placed the first two in 3rdparty/ (in the case of GeometricTools, what I have included is actually a small, modified subset of the original code).
However, Boost and Eigen are not included with this project's source; you must see  to it that CMake can find these two dependencies on your system (via find_package). 

Handling Boost is easy: you do not need to fully install it. Just download 
the Boost source, extract it to a directory, and set the Boost_INCLUDE_DIR CMake cache variable to point 
to the extracted source (specifically, to the directory containing boost/, doc/, libs/, tools/, and so on). 

Eigen, however, needs to be fully installed on your system in order for this project's CMake 
code to locate it. To install Eigen, run CMake on the Eigen source code to create a build system, build all,
and finally build the INSTALL target. If you are using Visual Studio on Windows, make sure to run Visual
Studio as administrator. The result of building the INSTALL target will be the creation of files at a CMake-known
system location: C:/Program Files (x86)/Eigen3/ in my case. The share/ folder at this location will contain files that let
my CMake code set up the proper connections to Eigen code. Note that the first time I tried to install 
Eigen, the share/ subdirectory did not appear, only include/. When I retried the process--creating a new Eigen.sln using CMake and then building the INSTALL 
target again--I saw the share/ folder appear.

## License
					
My own code--everything outside of 3rdparty/--is subject to the Boost Software License v1.0. See 
LICENSE.txt. Clipper2 (3rdparty/Clipper2Lib/) and GeometricTools (3rdparty/GeometricTools/) are also 
subject to the Boost Software License v1.0.
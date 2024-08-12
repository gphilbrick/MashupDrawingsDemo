add_executable( MashupDemo
	main.cpp 
	mashupinputs.h 
	prepareinputs.cpp 
	prepareinputs.h 
	savetofile.cpp
	savetofile.h
)

target_include_directories( MashupDemo
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries( MashupDemo PRIVATE Mashup::Mashup )
target_link_libraries( MashupDemo PRIVATE PrintCurves::PrintCurves )

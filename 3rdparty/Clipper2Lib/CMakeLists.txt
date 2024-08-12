project( Clipper2Lib )

add_library( ${PROJECT_NAME}
	include/clipper2/clipper.core.h
	include/clipper2/clipper.engine.h
	include/clipper2/clipper.export.h
	include/clipper2/clipper.h
	include/clipper2/clipper.minkowski.h
	include/clipper2/clipper.offset.h
	include/clipper2/clipper.rectclip.h
	include/clipper2/clipper.version.h	
	src/clipper.engine.cpp
	src/clipper.offset.cpp
	src/clipper.rectclip.cpp 
)
	
target_compile_features( ${PROJECT_NAME} PUBLIC cxx_std_17)
		
target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)	

add_library( ${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME} )



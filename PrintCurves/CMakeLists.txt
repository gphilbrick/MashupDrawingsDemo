project( PrintCurves )

add_library( ${PROJECT_NAME}
    PrintCurves/cap.h 
    PrintCurves/createfiles.cpp 
    PrintCurves/createfiles.h 
    PrintCurves/curvespostscript.cpp 
    PrintCurves/curvespostscript.h 
    PrintCurves/functors.h 
    PrintCurves/miteredcurve.cpp 
    PrintCurves/miteredcurve.h 
    PrintCurves/printcurvesutility.cpp
    PrintCurves/printcurvesutility.h 
    PrintCurves/strokeproperties.h
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}
	PRIVATE
		${PROJECT_SOURCE_DIR}/PrintCurves
)

target_link_libraries( ${PROJECT_NAME} PUBLIC Core::Core )

add_library( ${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME} )



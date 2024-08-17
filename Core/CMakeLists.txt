project (Core)

add_library( ${PROJECT_NAME} 
    Core/exceptions/runtimeerror.cpp 
    Core/exceptions/runtimeerror.h
    Core/math/canvasfield.cpp 
    Core/math/canvasfield.h
    Core/math/curveutility.cpp 
    Core/math/curveutility.h
    Core/math/interpcubic.cpp 
    Core/math/interpcubic.h
    Core/math/segcollidergrid.h
    Core/model/boundingboxback.h
    Core/model/boundingboxforward.h
    Core/model/curveback.h
    Core/model/curveforward.h 
    Core/model/curvesforward.h 
    Core/model/interval.cpp 
    Core/model/interval.h 
    Core/model/lineback.h 
    Core/model/lineforward.h 
    Core/model/polyline.h 
    Core/model/posback.h 
    Core/model/posforward.h 
    Core/model/rgbback.h 
    Core/model/rgbforward.h 
    Core/model/stroke.cpp 
    Core/model/stroke.h 
    Core/model/strokesforward.h 
    Core/model/stroketools.cpp 
    Core/model/stroketools.h 
    Core/utility/beziersfromspline.cpp 
    Core/utility/beziersfromspline.h 
    Core/utility/boundingbox.cpp 
    Core/utility/boundingbox.h 
    Core/utility/boundinginterval.cpp 
    Core/utility/boundinginterval.h 
    Core/utility/bspline2.cpp 
    Core/utility/bspline2.h 
    Core/utility/bspline2utility.cpp 
    Core/utility/bspline2utility.h 
    Core/utility/buildsplineexception.h 
    Core/utility/casts.h 
    Core/utility/curvecurveintersection.h 
    Core/utility/curvefitparametrizetype.h 
    Core/utility/curveinterval.cpp 
    Core/utility/curveinterval.h 
    Core/utility/curvesegment.cpp 
    Core/utility/curvesegment.h 
    Core/utility/ellipse.cpp 
    Core/utility/ellipse.h 
    Core/utility/intcoord.cpp 
    Core/utility/intcoord.h 
    Core/utility/intersectionparameters.h 
    Core/utility/linesegment.cpp 
    Core/utility/linesegment.h 
    Core/utility/mathutility.cpp 
    Core/utility/mathutility.h 
    Core/utility/polarinterval.cpp 
    Core/utility/polarinterval.h 
    Core/utility/twodarray.h 
    Core/utility/vector2.cpp 
    Core/utility/vector2.h 
    Core/utility/vector3.cpp 
    Core/utility/vector3.h 
    Core/utility/wall.cpp 
    Core/utility/wall.h 
    Core/view/consoleprogressbar.cpp 
    Core/view/consoleprogressbar.h 
    Core/view/progressbar.cpp
    Core/view/progressbar.h
)

target_compile_features( ${PROJECT_NAME} PUBLIC cxx_std_17 )

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}
	PRIVATE
		${PROJECT_SOURCE_DIR}/Core
)

find_package( Boost 1.64.0 REQUIRED )
find_package( Eigen3 3.3 REQUIRED NO_MODULE )
target_link_libraries( ${PROJECT_NAME} PUBLIC Boost::boost )
target_link_libraries( ${PROJECT_NAME} PRIVATE Eigen3::Eigen )
target_link_libraries( ${PROJECT_NAME} PRIVATE GeometricTools::GeometricTools )
target_link_libraries( ${PROJECT_NAME} PRIVATE Clipper2Lib::Clipper2Lib )

add_library( ${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME} )
project( Mashup )

add_library( ${PROJECT_NAME}
    Mashup/abfield.h 
    Mashup/blenddrawings.cpp 
    Mashup/blenddrawings.h 
    Mashup/blendoptions.h 
    Mashup/chains/chain.cpp 
    Mashup/chains/chain.h 
    Mashup/chains/chainbuilder.cpp 
    Mashup/chains/chainbuilder.h 
    Mashup/chains/joiner.cpp 
    Mashup/chains/joiner.h 
    Mashup/chains/joint.h 
    Mashup/chains/nextstep.cpp 
    Mashup/chains/nextstep.h 
    Mashup/chains/router.cpp 
    Mashup/chains/router.h 
    Mashup/drawing.cpp 
    Mashup/drawing.h 
    Mashup/drawingid.cpp 
    Mashup/drawingid.h 
    Mashup/drawings.cpp 
    Mashup/drawings.h 
    Mashup/endpoint.h 
    Mashup/onbarrierpath.cpp 
    Mashup/onbarrierpath.h 
    Mashup/paircutter.h 
    Mashup/randombinary.cpp 
    Mashup/randombinary.h 
    Mashup/samedrawinghits.cpp 
    Mashup/samedrawinghits.h 
    Mashup/scalarfield.cpp 
    Mashup/scalarfield.h 
    Mashup/strokeback.h 
    Mashup/strokeforward.h 
    Mashup/strokepoly.cpp 
    Mashup/strokepoly.h 
    Mashup/strokesegcollider.cpp 
    Mashup/strokesegcollider.h 
    Mashup/strokeside.h 
    Mashup/substroke.cpp 
    Mashup/substroke.h 
    Mashup/tails/taildata.cpp 
    Mashup/tails/taildata.h 
    Mashup/tails/tailmaker.cpp 
    Mashup/tails/tailmaker.h 
    Mashup/tails/tailtype.h 
    Mashup/tinterval.h 
    Mashup/topology/crossing.cpp 
    Mashup/topology/crossing.h 
    Mashup/topology/findtopology.cpp 
    Mashup/topology/findtopology.h 
    Mashup/topology/strokeintersection.h 
    Mashup/topology/strokeintervals.cpp 
    Mashup/topology/strokeintervals.h 
    Mashup/topology/topology.cpp
    Mashup/topology/topology.h
    Mashup/weightfunctor.cpp 
    Mashup/weightfunctor.h 
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}
	PRIVATE
		${PROJECT_SOURCE_DIR}/Mashup
)

target_link_libraries( ${PROJECT_NAME} PUBLIC Core::Core )
target_link_libraries( ${PROJECT_NAME} PRIVATE PrintCurves::PrintCurves )

add_library( ${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME} )


project( GeometricTools )

add_library( ${PROJECT_NAME} INTERFACE )

# We need both of these b/c GeometricTools internally #includes itself both ways.
target_include_directories( ${PROJECT_NAME}
    INTERFACE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/GTE
)

add_library( ${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME} )
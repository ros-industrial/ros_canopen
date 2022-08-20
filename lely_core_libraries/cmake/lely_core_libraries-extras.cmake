# dcfgen macro executes the dcfgen command for the specified target.
# For this macro to work there needs to be a folder config/{target}
# in your package. Inside that folder there needs to be the bus.yml
# file to use for generation.
macro(
    generate_dcf 
    TARGET)
    add_custom_target(
        ${TARGET}_prepare ALL 
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/*
        COMMAND rm -rf ${CMAKE_BINARY_DIR}/config/${TARGET}/*
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/config/${TARGET}
        COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/
    )

    add_custom_target(
        ${TARGET} ALL 
        DEPENDS ${TARGET}_prepare
    )

    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND dcfgen -d ${CMAKE_BINARY_DIR}/config/${TARGET}/ -rS bus.yml
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/
    )

    install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
    )
  
    install(DIRECTORY
        ${CMAKE_BINARY_DIR}/config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
    )

endmacro()

macro(dcfgen INPUT_DIR FILE OUTPUT_DIR)
    message(DEPRECATION "dcfgen macro is depreciated and will be remove in beta version. Use generate_dcf instead.")
    make_directory(${OUTPUT_DIR})
    add_custom_target(
        ${FILE} ALL
        COMMAND "dcfgen" "-d" ${OUTPUT_DIR} "-rS" ${INPUT_DIR}${FILE}
        WORKING_DIRECTORY ${INPUT_DIR}
        )
endmacro()
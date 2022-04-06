macro(dcfgen INPUT_DIR FILE OUTPUT_DIR)
make_directory(${OUTPUT_DIR})
    execute_process(
        COMMAND "dcfgen" "-d" ${OUTPUT_DIR} "-rS" ${INPUT_DIR}${FILE}
        WORKING_DIRECTORY ${INPUT_DIR}
        TIMEOUT 2
        RESULT_VARIABLE dcfgen_result
        )
endmacro()
# Define the source file relative path macro
function(define_rel_file_macro targetname)
    # Get current target all source files
    get_target_property(source_files "${targetname}" SOURCES)
    foreach(sourcefile ${source_files})
        # Gets the compilation parameters of the current source file
        get_property(defs SOURCE "${sourcefile}"
            PROPERTY COMPILE_DEFINITIONS)
        # Gets the absolute path to the current file
        get_filename_component(filepath "${sourcefile}" ABSOLUTE)
        # Replace the project path in the absolute path with empty to get the relative path of the source file with respect to the project path
        string(REPLACE ${PROJECT_SOURCE_DIR}/ "" relpath ${filepath})
        # Add the compile parameter we want to add (__REL_FILE__ definition) to the original compile parameter
        list(APPEND defs "__REL_FILE__=\"${relpath}\"")
        # Reset the compilation parameters of the source file
        set_property(
            SOURCE "${sourcefile}"
            PROPERTY COMPILE_DEFINITIONS ${defs}
        )
    endforeach()
endfunction()
find_package(Doxygen REQUIRED)

if (DOXYGEN_FOUND)
    # Doxygen configuration
    set(DOXYGEN_PROJECT_NAME Elite_Robots_CS_SDK)
    set(DOXYGEN_OPTIMIZE_OUTPUT_FOR_C YES)
    set(DOXYGEN_TYPEDEF_HIDES_STRUCT YES)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_EXTRACT_STATIC YES)
    set(DOXYGEN_STRIP_CODE_COMMENTS NO)
    set(DOXYGEN_MACRO_EXPANSION YES)
    set(DOXYGEN_EXPAND_ONLY_PREDEF YES)
    set(DOXYGEN_PREDEFINED ELITE_EXPORT)
    set(DOXYGEN_EXPAND_AS_DEFINED ELITE_EXPORT)
    set(DOXYGEN_COLLABORATION_GRAPH NO)
    set(DOXYGEN_INCLUDE_GRAPH NO)
    set(DOXYGEN_INCLUDED_BY_GRAPH NO)
    set(DOXYGEN_RECURSIVE NO)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
    set(DOXYGEN_GENERATE_XML YES)
    set(DOXYGEN_WARN_AS_ERROR FAIL_ON_WARNINGS) # Doxygen 1.9+
    set(DOXYGEN_ALIASES
        [[req="\xrefitem req \"Requirement\" \"Requirements\" "]]
        )
    set(DOXYGEN_VERBATIM_VARS DOXYGEN_ALIASES)
    set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/docs)

    doxygen_add_docs(
        doxygen_doc
        ${PROJECT_BINARY_DIR}/include/Elite/ 
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/doc/UserGuide/cn/
        ${CMAKE_SOURCE_DIR}/doc/UserGuide/en/
        ${CMAKE_SOURCE_DIR}/doc/Architecture/
        COMMENT "Generating documentation with Doxygen"
    )

    add_custom_target(
        doxygen_doc_target ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/doc/Doxyfile.doxygen_doc
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()

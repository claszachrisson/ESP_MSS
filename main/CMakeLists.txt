set(SOURCES main.cpp)
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS .
                    REQUIRES esp_mss)

#idf_build_get_property(project_dir PROJECT_DIR)
#idf_build_set_property(CONFIG_KCONFIG_FILES "${project_dir}/main/Kconfig" APPEND)
idf_build_set_property(CONFIG_KCONFIG_FILES "Kconfig" APPEND)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
add_executable(rtknavi_qt
    main.cpp
    mondlg.cpp
    instrdlg.cpp
    logstrdlg.cpp
    navimain.cpp
    outstrdlg.cpp
    rcvoptdlg.cpp
    #mapdlg.cpp
    markdlg.cpp

    instrdlg.ui
    logstrdlg.ui
    navimain.ui
    outstrdlg.ui
    rcvoptdlg.ui
    mondlg.ui
    #mapdlg.ui
    markdlg.ui

    ../appcmn_qt/aboutdlg.cpp
    ../appcmn_qt/cmdoptdlg.cpp
    ../appcmn_qt/fileoptdlg.cpp
    ../appcmn_qt/ftpoptdlg.cpp
    ../appcmn_qt/keydlg.cpp
    ../appcmn_qt/labelstretcher.cpp
    ../appcmn_qt/maskoptdlg.cpp
    ../appcmn_qt/refdlg.cpp
    ../appcmn_qt/serioptdlg.cpp
    ../appcmn_qt/mntpoptdlg.cpp
    ../appcmn_qt/tcpoptdlg.cpp
    ../appcmn_qt/viewer.cpp
    ../appcmn_qt/vieweropt.cpp
    ../appcmn_qt/freqdlg.cpp
    ../appcmn_qt/helper.cpp
    ../appcmn_qt/graph.cpp
    #../appcmn_qt/pntdlg.cpp
    ../appcmn_qt/navi_post_opt.cpp

    ../widgets_qt/doubleunitvalidator.cpp
    ../widgets_qt/intunitvalidator.cpp
    ../widgets_qt/scientificspinbox.cpp
    ../widgets_qt/excludedsatellitevalidator.cpp

    ../appcmn_qt/navi_post_opt.ui
    ../appcmn_qt/aboutdlg.ui
    ../appcmn_qt/cmdoptdlg.ui
    ../appcmn_qt/fileoptdlg.ui
    ../appcmn_qt/ftpoptdlg.ui
    ../appcmn_qt/keydlg.ui
    ../appcmn_qt/maskoptdlg.ui
    ../appcmn_qt/refdlg.ui
    ../appcmn_qt/serioptdlg.ui
    ../appcmn_qt/viewer.ui
    ../appcmn_qt/vieweropt.ui
    ../appcmn_qt/tcpoptdlg.ui
    ../appcmn_qt/mntpoptdlg.ui
    #../appcmn_qt/pntdlg.ui
    ../appcmn_qt/freqdlg.ui


    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
if(QT_VERSION GREATER_EQUAL 6.5)
        qt_add_translations(rtknavi_qt
		SOURCE_TARGETS rtknavi_qt
		INCLUDE_DIRECTORIES . ..appcmn_qt
        )
endif()
target_link_libraries(rtknavi_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::SerialPort
)

set_target_properties(rtknavi_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
install(TARGETS rtknavi_qt RUNTIME DESTINATION bin)
if (UNIX)
   install(FILES rtknavi_qt.desktop DESTINATION ${XDG_APPS_INSTALL_DIR})
   install(FILES ../icon/rtknavi.png DESTINATION ${XDG_APPS_PIXMAPS_DIR})
endif()

if(WIN32)
    # 复制运行库：librtklib.dll
    set(_rtklib_runtime_dll "${CMAKE_SOURCE_DIR}/lib/librtklib.dll")
    if(EXISTS "${_rtklib_runtime_dll}")
        add_custom_command(TARGET rtknavi_qt POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_rtklib_runtime_dll}"
                "$<TARGET_FILE_DIR:rtknavi_qt>"
                COMMENT "Copy librtklib.dll next to rtknavi_qt.exe"
                VERBATIM
        )
    else()
        message(WARNING "librtklib.dll not found at: ${_rtklib_runtime_dll}")
    endif()

    # 用 windeployqt 部署最小 Qt 运行时
    if(TARGET Qt6::qmake)
        get_target_property(_qmake_exe Qt6::qmake IMPORTED_LOCATION)
    elseif(TARGET Qt5::qmake)
        get_target_property(_qmake_exe Qt5::qmake IMPORTED_LOCATION)
    else()
        message(FATAL_ERROR "Qt qmake target not found. Ensure find_package(Qt5/Qt6 ...) was called.")
    endif()

    get_filename_component(_qt_bin_dir "${_qmake_exe}" DIRECTORY)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt_bin_dir}" REQUIRED)

    # MinGW runtime（libstdc++/libgcc/libwinpthread 等）需要编译器 bin 在 PATH
    get_filename_component(_toolchain_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)

    add_custom_command(TARGET rtknavi_qt POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E env
            "PATH=${_qt_bin_dir};${_toolchain_bin};$ENV{PATH}"
            "${WINDEPLOYQT_EXECUTABLE}"
            --no-translations
            --compiler-runtime
            --no-angle
            --no-opengl-sw
            "$<TARGET_FILE:rtknavi_qt>"
            COMMENT "Deploy minimal Qt runtime for rtknavi_qt.exe"
            VERBATIM
    )
endif()


# SPDX-License-Identifier: MIT

#[[
    This is a thin wrapper around CPM to reduce boilerplate.
    The primary purpose of this is to work with source archives
    rather than full Git clones, which are significantly faster
    to fetch and take up less overall space.

    This module is loosely based on CPMUtil, which can be found at
    https://git.crueter.xyz/CMake/CPMUtil. This project and its derivatives are
    granted an exclusive exception to the terms of the LGPLv3, and may use,
    reference, and adapt any code from CPMUtil for the purposes of this module at
    will.
]]

set(CPM_SOURCE_CACHE ${PROJECT_SOURCE_DIR}/.cache/cpm)

include(CPM)

macro(Propagate var)
    set(${var} ${${var}} PARENT_SCOPE)
endmacro()

# TODO: Handle system packages
function(AddPackage)
    set(oneValueArgs
        NAME
        HASH
        REPO
        VERSION
        SOURCE_SUBDIR)

    set(multiValueArgs OPTIONS)

    set(options)

    cmake_parse_arguments(PKG "${options}" "${oneValueArgs}" "${multiValueArgs}"
        "${ARGN}")

    if (NOT DEFINED PKG_NAME)
        message(FATAL_ERROR "[AddPackage] NAME is required")
    endif()

    if (NOT DEFINED PKG_REPO)
        message(FATAL_ERROR "[AddPackage] REPO is required")
    endif()

    if (NOT DEFINED PKG_REPO)
        message(FATAL_ERROR "[AddPackage] VERSION is required")
    endif()


    # TODO(crueter): forgejo
    set(git_url https://github.com/${PKG_REPO})

    set(pkg_url ${git_url}/archive/${PKG_VERSION}.tar.gz)

    set(CPM_USE_LOCAL_PACKAGES OFF)

    CPMAddPackage(
        NAME ${PKG_NAME}
        URL ${pkg_url}
        URL_HASH ${PKG_HASH}
        VERSION ${_version}
        CUSTOM_CACHE_KEY ${key}

        OPTIONS ${PKG_OPTIONS}
        PATCHES ${PKG_PATCHES}
        EXCLUDE_FROM_ALL ON
        SOURCE_SUBDIR ${PKG_SOURCE_SUBDIR})

    Propagate(${PKG_NAME}_ADDED)
    Propagate(${PKG_NAME}_SOURCE_DIR)
endfunction()

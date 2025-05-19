include(FetchContent)
FetchContent_Declare(
	asio-repo
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/asio
	FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(asio-repo)

file(GLOB_RECURSE asioHeaders_glob
		"${asio-repo_SOURCE_DIR}/asio/include/*.hpp"
		"${asio-repo_SOURCE_DIR}/asio/include/*.h"
		"${asio-repo_SOURCE_DIR}/asio/include/*.ipp")

target_sources(orcaSDK_core
	PUBLIC
		FILE_SET asioHeaders
		TYPE HEADERS
		BASE_DIRS "${asio-repo_SOURCE_DIR}/asio/include"
		FILES
			${asioHeaders_glob}
)
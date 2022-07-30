load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "lib_socket",
    srcs = [
        "socket.cc",
    ],
    hdrs = [
        "socket.h",
    ],
    deps = [
        "//modules/localization/proto:localization_proto",
        "//modules/server/rosnode:lib_rosnode",
        "//modules/server/proto:localization_gnss_status_proto",
    ],
    linkopts = ["-lpthread"],
)

cpplint()
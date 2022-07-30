load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "lib_server",
    srcs = [
        "server.cc",
    ],
    hdrs = [
        "server.h",
    ],
    deps = [
        "//modules/localization/proto:localization_proto",
        "//modules/server/rosnode:lib_rosnode",
        "//modules/server/proto:localization_gnss_status_proto",
    ],
    linkopts = ["-lpthread"],
)

cpplint()
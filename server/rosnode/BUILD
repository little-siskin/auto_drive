load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_rosnode",
    srcs = [
        "rosnode.cc",
    ],
    hdrs = [
        "rosnode.h",
    ],
    deps = [
        "//modules/common/proto:gnss_status_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/canbus/proto:canbus_proto",
        "//third_party/ros:ros_common",
    ],
    linkopts = ["-lpthread"],
)

cpplint()
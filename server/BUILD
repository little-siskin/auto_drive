load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "server",
    srcs = [
        "main.cc",
    ],
    deps = [
        "//modules/server/sub_server:lib_server",
        "//external:gflags",
        "//modules/common:log",
    ],
)


cpplint()
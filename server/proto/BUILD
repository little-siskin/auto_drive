load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "localization_gnss_status_proto",
    protos = ["localization_gnss_status.proto"],
    deps = [
        "//modules/common/proto:gnss_status_proto",
        "//modules/localization/proto:localization_proto",
    ],
)

py_proto_compile(
    name = "localization_gnss_status_proto_pylib",
    protos = ["localization_gnss_status.proto"],
    deps = [
        "//modules/common/proto:common_proto_pylib",
        "//modules/localization/proto:localization_proto_pylib",
    ],
)

cpplint()
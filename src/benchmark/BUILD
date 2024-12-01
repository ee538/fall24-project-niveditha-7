load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "bench_mark",
    srcs = ["bench_mark.cc"],
    deps = [
        "@com_google_benchmark//:benchmark",
        "//src/lib:TrojanMap",
    ],
)

load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "tsp_benchmark",
    srcs = ["tsp_benchmark.cc"],
    deps = [
        "@com_google_benchmark//:benchmark",
        "//src/lib:TrojanMap",  
    ],
)



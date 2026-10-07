"""Use -Og for the entry/actuator modules; retain -O2 for other modules."""
Import("env")


def debug_modules(build_env, node):
    source = node.srcnode().get_abspath().replace("\\", "/")
    project = build_env.subst("$PROJECT_DIR").replace("\\", "/")
    selected = {project + "/src/main.cpp", project + "/src/actuator_output.cpp"}
    if source not in selected:
        return node
    flags = [flag for flag in build_env.get("CCFLAGS", [])
             if str(flag) not in ("-O0", "-O1", "-O2", "-O3", "-Og", "-Os", "-Ofast")]
    return build_env.Object(node, CCFLAGS=flags + ["-Og"])


env.AddBuildMiddleware(debug_modules)

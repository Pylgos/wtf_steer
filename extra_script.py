Import("env")
env.Append(CXXFLAGS=["-Wno-register"])
# include toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# override compilation DB path
# env.Replace(COMPILATIONDB_PATH=os.path.join("$BUILD_DIR", "compile_commands.json"))
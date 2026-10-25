# Link the native-san env against the sanitizer runtimes (build_flags only reach the compiler).
Import("env")
env.Append(LINKFLAGS=["-fsanitize=address,undefined"])

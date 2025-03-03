Import("env")

cxx = env["CXXFLAGS"]
cxx = [x for x in cxx if not "-std" in x]
cxx += ["-std=c++2b"]
env.Replace(CXXFLAGS=cxx)
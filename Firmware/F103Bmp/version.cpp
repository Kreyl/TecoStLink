#define STRINGIFY(x)    # x
#define XSTRINGIFY(x)   STRINGIFY(x)

const char* kBuildTime = XSTRINGIFY(BUILD_TIME);
const char* kBuildCfgName = XSTRINGIFY(BUILD_CFG);
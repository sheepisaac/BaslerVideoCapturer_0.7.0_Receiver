#include <iostream>
#include <string>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/DeviceInfo.h>

using namespace std;
using namespace Pylon;

extern vector<String_t> camNumList = {	"40270145", "40270147", "40270148", "40270149", "40270150",
										"40270151", "40270153", "40270154", "40270159", "40270160",
										"40275506", "40275521", "40315370", "40315373", "40315377",
										"40315379", "40315440", "40315441", "40333061", "40341697"};
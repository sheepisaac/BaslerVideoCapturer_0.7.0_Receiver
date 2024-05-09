// Contains a configuration that sets pixel data format and Image AOI.

#ifndef INCLUDED_PIXELFORMATANDAOICONFIGURATION_H_00104928
#define INCLUDED_PIXELFORMATANDAOICONFIGURATION_H_00104928

#include <pylon/PylonIncludes.h>
#include <pylon/ConfigurationEventHandler.h>
#include <pylon/ParameterIncludes.h>

extern size_t num_exposure, num_fps;
extern size_t option_lightSource, option_balanceWhiteAuto, option_deMosaicingMode;
extern float num_noise, num_sharpness;

namespace Pylon {
    class CInstantCamera;
}
class CPixelFormatAndAoiConfiguration : public Pylon::CConfigurationEventHandler {
public:
    void OnOpened(Pylon::CInstantCamera& camera) {
        try {
            using namespace Pylon;
            GenApi::INodeMap& nodemap = camera.GetNodeMap();
            CIntegerParameter width(nodemap, "Width");
            CIntegerParameter height(nodemap, "Height");
            CIntegerParameter offsetX(nodemap, "OffsetX");
            CIntegerParameter offsetY(nodemap, "OffsetY");

            width.SetValue(1952);
            height.SetValue(1088);
            offsetX.SetValue(320);
            offsetY.SetValue(480);

            CEnumParameter(nodemap, "PixelFormat").SetValue("RGB8");
            CFloatParameter(nodemap, "ExposureTime").SetValue(num_exposure);
            CBooleanParameter(nodemap, "AcquisitionFrameRateEnable").SetValue(true);
            CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(num_fps);

            switch (option_lightSource) {
            case 0: CEnumParameter(nodemap, "LightSourcePreset").SetValue("Off"); break;
            case 1: CEnumParameter(nodemap, "LightSourcePreset").SetValue("Daylight5000K"); break;
            case 2: CEnumParameter(nodemap, "LightSourcePreset").SetValue("Daylight6400K"); break;
            case 3: CEnumParameter(nodemap, "LightSourcePreset").SetValue("Tungsten2800K"); break;
            default: break;
            }

            switch (option_balanceWhiteAuto) {
            case 0: CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue("Off"); break;
            case 1: CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue("Once"); break;
            case 2: CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue("Continuous"); break;
            default: break;
            }

            if (option_deMosaicingMode == 1) {
                CEnumParameter(nodemap, "DemosaicingMode").SetValue("BaslerPGI");
                CFloatParameter(nodemap, "NoiseReduction").SetValue(num_noise);
                CFloatParameter(nodemap, "SharpnessEnhancement").SetValue(num_sharpness);
            }
        }
        catch (const Pylon::GenericException& e) {
            throw RUNTIME_EXCEPTION("Could not apply configuration. const GenericException caught in OnOpened method msg = %hs", e.what());
        }
    }
};


#endif /* INCLUDED_PIXELFORMATANDAOICONFIGURATION_H_00104928 */

#pragma once

#include "d3d-render-utils/RenderPipeline.h"
#include <wrl/client.h>

class FFR {
public:
    FFR(ID3D11Device* device);
    void Initialize(ID3D11Texture2D* compositionTexture);
    void Render();
    void GetOptimizedResolution(uint32_t* width, uint32_t* height);
    ID3D11Texture2D* GetOutputTexture();

private:
    Microsoft::WRL::ComPtr<ID3D11Device> mDevice;
    Microsoft::WRL::ComPtr<ID3D11Texture2D> mOptimizedTexture;
    Microsoft::WRL::ComPtr<ID3D11VertexShader> mQuadVertexShader;
    Microsoft::WRL::ComPtr<ID3D11Buffer> mFoveationBuffer;

    bool m_gazeEnabled = false;
    float m_gazeSmoothing = 0.2f;
    float m_fallbackCenterX = 0.5f;
    float m_fallbackCenterY = 0.5f;

    std::vector<d3d_render_utils::RenderPipeline> mPipelines;
};

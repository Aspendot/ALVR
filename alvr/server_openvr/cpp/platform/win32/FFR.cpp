#include "FFR.h"

#include "GazeStore.h"
#include "alvr_server/Settings.h"
#include "alvr_server/Utils.h"
#include "alvr_server/bindings.h"
#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>

using Microsoft::WRL::ComPtr;
using namespace d3d_render_utils;

namespace {

class GazeListener {
public:
    void Start(uint16_t port, float smoothing) {
        m_smoothing = smoothing;
        if (m_running.exchange(true)) {
            return;
        }

        m_thread = std::thread([this, port]() {
            WSADATA wsaData;
            WSAStartup(MAKEWORD(2, 2), &wsaData);

            SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock == INVALID_SOCKET) {
                m_running = false;
                return;
            }

            sockaddr_in addr {};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            addr.sin_port = htons(port);

            if (bind(sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
                closesocket(sock);
                m_running = false;
                return;
            }

            u_long nonBlocking = 1;
            ioctlsocket(sock, FIONBIO, &nonBlocking);

            while (m_running.load()) {
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(sock, &readfds);

                timeval tv {};
                tv.tv_sec = 0;
                tv.tv_usec = 5'000; // 5 ms

                if (select(0, &readfds, NULL, NULL, &tv) > 0 && FD_ISSET(sock, &readfds)) {
                    float coords[2] = { 0.5f, 0.5f };
                    int recvLen = recvfrom(sock, (char*)coords, sizeof(coords), 0, NULL, NULL);
                    if (recvLen >= (int)sizeof(coords)) {
                        for (int i = 0; i < 2; ++i) {
                            if (!std::isfinite(coords[i])) {
                                coords[i] = 0.5f;
                            }
                            coords[i] = std::clamp(coords[i], 0.0f, 1.0f);
                        }
                        const float alpha = std::clamp(m_smoothing, 0.0f, 1.0f);
                        m_x.store(m_x.load() + (coords[0] - m_x.load()) * alpha);
                        m_y.store(m_y.load() + (coords[1] - m_y.load()) * alpha);
                        alvr::set_gaze_sample(m_x.load(), m_y.load());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            closesocket(sock);
            WSACleanup();
        });
        m_thread.detach();
    }

    std::pair<float, float> Sample(float fallbackX, float fallbackY) const {
        if (!m_running.load()) {
            return { fallbackX, fallbackY };
        }
        return { m_x.load(), m_y.load() };
    }

private:
    std::atomic<bool> m_running { false };
    std::atomic<float> m_x { 0.5f };
    std::atomic<float> m_y { 0.5f };
    float m_smoothing = 0.2f;
    std::thread m_thread;
};

static GazeListener g_gazeListener;

struct FoveationVars {
    uint32_t targetEyeWidth;
    uint32_t targetEyeHeight;
    uint32_t optimizedEyeWidth;
    uint32_t optimizedEyeHeight;

    float eyeWidthRatio;
    float eyeHeightRatio;

    float centerSizeX;
    float centerSizeY;
    float centerShiftX;
    float centerShiftY;
    float edgeRatioX;
    float edgeRatioY;
};

FoveationVars CalculateFoveationVars(float centerShiftXOverride, float centerShiftYOverride) {
    float targetEyeWidth = (float)Settings::Instance().m_renderWidth / 2;
    float targetEyeHeight = (float)Settings::Instance().m_renderHeight;

    float centerSizeX = (float)Settings::Instance().m_foveationCenterSizeX;
    float centerSizeY = (float)Settings::Instance().m_foveationCenterSizeY;
    float centerShiftX = centerShiftXOverride;
    float centerShiftY = centerShiftYOverride;
    float edgeRatioX = (float)Settings::Instance().m_foveationEdgeRatioX;
    float edgeRatioY = (float)Settings::Instance().m_foveationEdgeRatioY;

    float edgeSizeX = targetEyeWidth - centerSizeX * targetEyeWidth;
    float edgeSizeY = targetEyeHeight - centerSizeY * targetEyeHeight;

    float centerSizeXAligned
        = 1. - ceil(edgeSizeX / (edgeRatioX * 2.)) * (edgeRatioX * 2.) / targetEyeWidth;
    float centerSizeYAligned
        = 1. - ceil(edgeSizeY / (edgeRatioY * 2.)) * (edgeRatioY * 2.) / targetEyeHeight;

    float edgeSizeXAligned = targetEyeWidth - centerSizeXAligned * targetEyeWidth;
    float edgeSizeYAligned = targetEyeHeight - centerSizeYAligned * targetEyeHeight;

    float centerShiftXAligned = ceil(centerShiftX * edgeSizeXAligned / (edgeRatioX * 2.))
        * (edgeRatioX * 2.) / edgeSizeXAligned;
    float centerShiftYAligned = ceil(centerShiftY * edgeSizeYAligned / (edgeRatioY * 2.))
        * (edgeRatioY * 2.) / edgeSizeYAligned;

    float foveationScaleX = (centerSizeXAligned + (1. - centerSizeXAligned) / edgeRatioX);
    float foveationScaleY = (centerSizeYAligned + (1. - centerSizeYAligned) / edgeRatioY);

    float optimizedEyeWidth = foveationScaleX * targetEyeWidth;
    float optimizedEyeHeight = foveationScaleY * targetEyeHeight;

    // round the frame dimensions to a number of pixel multiple of 32 for the encoder
    auto optimizedEyeWidthAligned = (uint32_t)ceil(optimizedEyeWidth / 32.f) * 32;
    auto optimizedEyeHeightAligned = (uint32_t)ceil(optimizedEyeHeight / 32.f) * 32;

    float eyeWidthRatioAligned = optimizedEyeWidth / optimizedEyeWidthAligned;
    float eyeHeightRatioAligned = optimizedEyeHeight / optimizedEyeHeightAligned;

    return { (uint32_t)targetEyeWidth,
             (uint32_t)targetEyeHeight,
             optimizedEyeWidthAligned,
             optimizedEyeHeightAligned,
             eyeWidthRatioAligned,
             eyeHeightRatioAligned,
             centerSizeXAligned,
             centerSizeYAligned,
             centerShiftXAligned,
             centerShiftYAligned,
             edgeRatioX,
             edgeRatioY };
}
}

void FFR::GetOptimizedResolution(uint32_t* width, uint32_t* height) {
    auto fovVars = CalculateFoveationVars(
        (float)Settings::Instance().m_foveationCenterShiftX,
        (float)Settings::Instance().m_foveationCenterShiftY
    );
    *width = fovVars.optimizedEyeWidth * 2;
    *height = fovVars.optimizedEyeHeight;
}

FFR::FFR(ID3D11Device* device)
    : mDevice(device) { }

void FFR::Initialize(ID3D11Texture2D* compositionTexture) {
    m_gazeEnabled = Settings::Instance().m_gazeStreamEnabled;
    m_gazeSmoothing = Settings::Instance().m_gazeSmoothingFactor;
    m_fallbackCenterX = Settings::Instance().m_gazeFallbackCenterX;
    m_fallbackCenterY = Settings::Instance().m_gazeFallbackCenterY;
    if (m_gazeEnabled) {
        g_gazeListener.Start(Settings::Instance().m_gazeUdpPort, m_gazeSmoothing);
    }

    auto fovVars = CalculateFoveationVars(
        (float)Settings::Instance().m_foveationCenterShiftX,
        (float)Settings::Instance().m_foveationCenterShiftY
    );
    mFoveationBuffer = CreateBuffer(mDevice.Get(), fovVars, D3D11_USAGE_DYNAMIC);

    std::vector<uint8_t> quadShaderCSO(
        QUAD_SHADER_CSO_PTR, QUAD_SHADER_CSO_PTR + QUAD_SHADER_CSO_LEN
    );
    mQuadVertexShader = CreateVertexShader(mDevice.Get(), quadShaderCSO);

    mOptimizedTexture = CreateTexture(
        mDevice.Get(),
        fovVars.optimizedEyeWidth * 2,
        fovVars.optimizedEyeHeight,
        Settings::Instance().m_enableHdr ? DXGI_FORMAT_R16G16B16A16_FLOAT
                                         : DXGI_FORMAT_R8G8B8A8_UNORM_SRGB
    );

    if (Settings::Instance().m_enableFoveatedEncoding) {
        std::vector<uint8_t> compressAxisAlignedShaderCSO(
            COMPRESS_AXIS_ALIGNED_CSO_PTR,
            COMPRESS_AXIS_ALIGNED_CSO_PTR + COMPRESS_AXIS_ALIGNED_CSO_LEN
        );
        auto compressAxisAlignedPipeline = RenderPipeline(mDevice.Get());
        compressAxisAlignedPipeline.Initialize(
            { compositionTexture },
            mQuadVertexShader.Get(),
            compressAxisAlignedShaderCSO,
            mOptimizedTexture.Get(),
            mFoveationBuffer.Get()
        );

        mPipelines.push_back(compressAxisAlignedPipeline);
    } else {
        mOptimizedTexture = compositionTexture;
    }
}

void FFR::Render() {
    if (m_gazeEnabled) {
        auto gaze = g_gazeListener.Sample(m_fallbackCenterX, m_fallbackCenterY);
        alvr::set_gaze_sample(gaze.first, gaze.second);
        float shiftX = (gaze.first - 0.5f) * 2.0f;
        float shiftY = (gaze.second - 0.5f) * 2.0f;
        auto vars = CalculateFoveationVars(shiftX, shiftY);

        ComPtr<ID3D11DeviceContext> context;
        mDevice->GetImmediateContext(&context);
        D3D11_MAPPED_SUBRESOURCE mapped {};
        if (context
            && SUCCEEDED(
                context->Map(
                    mFoveationBuffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped
                )
            )) {
            memcpy(mapped.pData, &vars, sizeof(vars));
            context->Unmap(mFoveationBuffer.Get(), 0);
        }
    } else if (!mFoveationBuffer) {
        auto vars = CalculateFoveationVars(
            (float)Settings::Instance().m_foveationCenterShiftX,
            (float)Settings::Instance().m_foveationCenterShiftY
        );
        mFoveationBuffer = CreateBuffer(mDevice.Get(), vars, D3D11_USAGE_DYNAMIC);
    }

    for (auto& p : mPipelines) {
        p.Render();
    }
}

ID3D11Texture2D* FFR::GetOutputTexture() { return mOptimizedTexture.Get(); }

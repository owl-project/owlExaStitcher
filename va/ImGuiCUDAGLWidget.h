// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <functional>
#include <cuda_runtime_api.h>
#include <GL/gl.h>
#include <owl/common/math/vec.h>

namespace exa
{
    typedef std::function<void(int32_t x, int32_t y, int32_t button)> ImGuiMouseFunc;

    class ImGuiCUDAGLWidget
    {
    public:
       ~ImGuiCUDAGLWidget();

        //! Render with ImGui, open in new ImgGui window
        void show();

        //! Render with ImGui but w/o window
        void drawImmediate();

        //! Map graphics resource and return a surface object to write to
        cudaSurfaceObject_t map();

        //! Unmap graphics resource
        void unmap();

        // Resize canvas
        void resize(int w, int h);

        //! Return canvas width
        int width() const
        { return canvasSize_.x; }

        //! Return canvas height
        int height() const
        { return canvasSize_.y; }

        //! Called when mouse moved and no button is pressed
        void setPassiveMotionFunc(ImGuiMouseFunc func)
        { passiveMotionFunc = func; }

        //! Called when mouse moved and a button is pressed
        void setMotionFunc(ImGuiMouseFunc func)
        { motionFunc = func; }

        //! Called when a button was pressed
        void setPressFunc(ImGuiMouseFunc func)
        { pressFunc = func; }

        //! Called when a button was released
        void setReleaseFunc(ImGuiMouseFunc func)
        { releaseFunc = func; }

    private:
        // RGBA texture
        GLuint texture_ = GLuint(-1);

        // CUDA graphics resource
        cudaGraphicsResource_t resource_{0};

        // Drawing canvas size
        owl::vec2i canvasSize_ = { 300, 150 };

        // Mouse state for drawing
        struct MouseEvent
        {
            enum Type { PassiveMotion, Motion, Press, Release };
            enum Button { Left, Middle, Right, None };

            owl::vec2i pos = { 0, 0 };
            int button = None;
            Type type = Motion;
        };

        // The last mouse event
        MouseEvent lastEvent_;

        // Dragging in progress
        bool dragging_ = false;

        // Create GL texture
        void createTexture();

        // Generate mouse event when mouse hovered over rect
        MouseEvent generateMouseEvent();

        // Handle mouse event
        void handleMouseEvent(MouseEvent const& event);

        // Called when mouse moved and no button is pressed
        ImGuiMouseFunc passiveMotionFunc = nullptr;

        // Called when mouse moved and a button is pressed
        ImGuiMouseFunc motionFunc = nullptr;

        // Called when a button was pressed
        ImGuiMouseFunc pressFunc = nullptr;

        // Called when a button was released
        ImGuiMouseFunc releaseFunc = nullptr;
    };

} // exa

// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <cuda_runtime_api.h>
#include <GL/gl.h>
#include <owl/common/math/vec.h>

namespace exa
{
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
    };

} // exa

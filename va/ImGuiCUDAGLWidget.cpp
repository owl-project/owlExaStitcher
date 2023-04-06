// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <iostream>
#include <GL/glew.h>
#include <cuda_gl_interop.h>
#include <imgui.h>
#include "ImGuiCUDAGLWidget.h"

#define cudaCheckError() {                                          \
 cudaError_t e=cudaGetLastError();                                 \
 if(e!=cudaSuccess) {                                              \
   printf("Cuda failure %s:%d: '%s'\n",__FILE__,__LINE__,cudaGetErrorString(e));           \
   exit(0); \
 }                                                                 \
}

static void enableBlendCB(ImDrawList const*, ImDrawCmd const*)
{
    glEnable(GL_BLEND);
}

static void disableBlendCB(ImDrawList const*, ImDrawCmd const*)
{
    glDisable(GL_BLEND);
}

namespace exa
{
    ImGuiCUDAGLWidget::~ImGuiCUDAGLWidget()
    {
        // TODO: cannot be sure we have a GL context here!
        glDeleteTextures(1, &texture_);
    }

    void ImGuiCUDAGLWidget::show()
    {
        ImGui::Begin("<Window Title>");

        drawImmediate();

        ImGui::End();
    }

    void ImGuiCUDAGLWidget::drawImmediate()
    {
        if (0) // mapped with CUDA
            return;

        if (texture_ == GLuint(-1))
            createTexture();

        ImGui::GetWindowDrawList()->AddCallback(disableBlendCB, nullptr);
        ImGui::ImageButton(
            (void*)(intptr_t)texture_,
            ImVec2(canvasSize_.x, canvasSize_.y),
            ImVec2(0, 0),
            ImVec2(1, 1),
            0 // frame size = 0
            );

        MouseEvent event = generateMouseEvent();
        handleMouseEvent(event);

        ImGui::GetWindowDrawList()->AddCallback(enableBlendCB, nullptr);
    }

    cudaSurfaceObject_t ImGuiCUDAGLWidget::map() {
        if (texture_ == GLuint(-1))
            createTexture(); // also initializes resource

        cudaGraphicsMapResources(1, &resource_);

        cudaSurfaceObject_t surfaceObj;
        cudaResourceDesc desc;
        memset(&desc, 0, sizeof(desc));
        cudaArray_t array;
        cudaGraphicsSubResourceGetMappedArray(
            &array,
            resource_,
            0,
            0);
        desc.resType = cudaResourceTypeArray;
        desc.res.array.array = array;
        cudaCreateSurfaceObject(&surfaceObj, &desc);
        return surfaceObj;
    }

    void ImGuiCUDAGLWidget::unmap()
    {
        cudaGraphicsUnmapResources(1, &resource_);
    }

    void ImGuiCUDAGLWidget::createTexture()
    {
        glGenTextures(1, &texture_);

        GLint prevTexture;
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &prevTexture);
        glBindTexture(GL_TEXTURE_2D, texture_);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA32F,
            canvasSize_.x,
            canvasSize_.y,
            0,
            GL_RGBA,
            GL_FLOAT,
            nullptr
            );

        cudaGraphicsGLRegisterImage(
            &resource_,
            texture_,
            GL_TEXTURE_2D,
            cudaGraphicsRegisterFlagsSurfaceLoadStore
            );

        glBindTexture(GL_TEXTURE_2D, prevTexture);
    }

    ImGuiCUDAGLWidget::MouseEvent ImGuiCUDAGLWidget::generateMouseEvent()
    {
        MouseEvent event;

        int x = ImGui::GetIO().MousePos.x - ImGui::GetCursorScreenPos().x;
        int y = ImGui::GetCursorScreenPos().y - ImGui::GetIO().MousePos.y - 1;
        
        event.pos = { x, y };
        event.button = ImGui::GetIO().MouseDown[0] ? MouseEvent::Left :
                       ImGui::GetIO().MouseDown[1] ? MouseEvent::Middle :
                       ImGui::GetIO().MouseDown[2] ? MouseEvent::Right:
                                                     MouseEvent::None;
        // TODO: handle the unlikely case that the down button is not
        // the same as the one from lastEvent_. This could happen as
        // the mouse events are tied to the rendering frame rate
        if (event.button == MouseEvent::None && lastEvent_.button == MouseEvent::None)
            event.type = MouseEvent::PassiveMotion;
        else if (event.button != MouseEvent::None && lastEvent_.button != MouseEvent::None)
            event.type = MouseEvent::Motion;
        else if (event.button != MouseEvent::None && lastEvent_.button == MouseEvent::None)
            event.type = MouseEvent::Press;
        else
            event.type = MouseEvent::Release;

        return event;
    }

    void ImGuiCUDAGLWidget::handleMouseEvent(ImGuiCUDAGLWidget::MouseEvent const& event)
    {
        bool hovered = ImGui::IsItemHovered()
                && event.pos.x >= 0 && event.pos.x < canvasSize_.x
                && event.pos.y >= 0 && event.pos.y < canvasSize_.y;

        if (event.type == MouseEvent::PassiveMotion || event.type == MouseEvent::Release)
            dragging_ = false;

        if (dragging_ || (event.type == MouseEvent::Press && hovered && event.button == MouseEvent::Left))
        {
            dragging_ = true;
        }

        lastEvent_ = event;
    }

} // exa

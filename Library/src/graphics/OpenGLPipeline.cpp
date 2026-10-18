/*    
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  OpenGLPipeline.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 30/03/2014.
//  Copyright(c) 2014-2026 Patryk Cieslak. All rights reserved.
//

#include "graphics/OpenGLPipeline.h"

#include <algorithm>
#include "core/SimulationManager.h"
#include "graphics/OpenGLState.h"
#include "graphics/GLSLShader.h"
#include "graphics/OpenGLContent.h"
#include "graphics/OpenGLOcean.h"
#include "graphics/OpenGLTrackball.h"
#include "graphics/OpenGLCamera.h"
#include "graphics/OpenGLRealCamera.h"
#include "graphics/OpenGLDepthCamera.h"
#include "graphics/OpenGLThermalCamera.h"
#include "graphics/OpenGLOpticalFlowCamera.h"
#include "graphics/OpenGLSegmentationCamera.h"
#include "graphics/OpenGLEventBasedCamera.h"
#include "graphics/OpenGLSonar.h"
#include "graphics/OpenGLAtmosphere.h"
#include "graphics/OpenGLLight.h"
#include "graphics/OpenGLOceanParticles.h"
#include "utils/SystemUtil.hpp"
#include "entities/forcefields/Ocean.h"
#include "entities/forcefields/Atmosphere.h"
#include "core/GraphicalSimulationApp.h"

namespace sf
{

OpenGLPipeline::OpenGLPipeline(RenderSettings s, HelperSettings h) : rSettings_(s), hSettings_(h)
{
    drawingQueueMutex_ = SDL_CreateMutex();
    
    //Set default OpenGL options
    cInfo("Initialising OpenGL rendering pipeline...");
     
    //Load shaders and create rendering buffers
    cInfo("Loading shaders...");
    OpenGLAtmosphere::Init();
    OpenGLCamera::Init(rSettings_);
    OpenGLDepthCamera::Init();
    OpenGLThermalCamera::Init();
    OpenGLOpticalFlowCamera::Init();
    OpenGLSegmentationCamera::Init();
    OpenGLEventBasedCamera::Init();
    OpenGLSonar::Init();
    OpenGLOceanParticles::Init();
    content_ = std::make_unique<OpenGLContent>();
    
    //Create display framebuffer
    glGenFramebuffers(1, &screenFBO_);
    OpenGLState::BindFramebuffer(screenFBO_);
    glGenTextures(1, &screenTex_);
    OpenGLState::BindTexture(TEX_BASE, GL_TEXTURE_2D, screenTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, rSettings_.windowW, rSettings_.windowH, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //Cheaper/better gaussian blur for GUI background
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //Cheaper/better gaussian blur for GUI background
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, screenTex_, 0);
    OpenGLState::UnbindTexture(TEX_BASE);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(status != GL_FRAMEBUFFER_COMPLETE)
        cError("Display FBO initialization failed!");
    OpenGLState::BindFramebuffer(0);
    lastSimTime_ = Scalar(-1);
    causticsDt_ = Scalar(1); //Caustics generated in the first frame
    windowMinimized_ = false;
    framesSinceFast_ = 0;
}

OpenGLPipeline::~OpenGLPipeline()
{
    OpenGLCamera::Destroy();
    OpenGLDepthCamera::Destroy();
    OpenGLThermalCamera::Destroy();
    OpenGLOpticalFlowCamera::Destroy();
    OpenGLSegmentationCamera::Destroy();
    OpenGLEventBasedCamera::Destroy();
    OpenGLSonar::Destroy();
    OpenGLOceanParticles::Destroy();
    OpenGLLight::Destroy();
    
    glDeleteTextures(1, &screenTex_);
    glDeleteFramebuffers(1, &screenFBO_);
    SDL_DestroyMutex(drawingQueueMutex_);
}

RenderSettings OpenGLPipeline::GetRenderSettings() const
{
    return rSettings_;
}
    
HelperSettings& OpenGLPipeline::GetHelperSettings()
{
    return hSettings_;
}
    
GLuint OpenGLPipeline::GetScreenTexture()
{
    return screenTex_;
}

SDL_mutex* OpenGLPipeline::GetDrawingQueueMutex()
{
    return drawingQueueMutex_;
}
    
void OpenGLPipeline::SetWindowMinimized(bool minimized)
{
    windowMinimized_ = minimized;
}

OpenGLContent* OpenGLPipeline::GetContent()
{
    return content_.get();
}

void OpenGLPipeline::AddToDrawingQueue(const Renderable& r)
{
    drawingQueue_.push_back(r);
}

void OpenGLPipeline::AddToDrawingQueue(const std::vector<Renderable>& r)
{
    drawingQueue_.insert(drawingQueue_.end(), r.begin(), r.end());
}

void OpenGLPipeline::AddToSelectedDrawingQueue(const std::vector<Renderable>& r)
{
    selectedDrawingQueue_.insert(selectedDrawingQueue_.end(), r.begin(), r.end());
}

void OpenGLPipeline::PurgeDrawingQueue()
{
    drawingQueue_.clear();
}

void OpenGLPipeline::PurgeSelectedDrawingQueue()
{
    selectedDrawingQueue_.clear();
}

bool OpenGLPipeline::IsDrawingQueueEmpty()
{
    return drawingQueue_.empty();
}
    
void OpenGLPipeline::PerformDrawingQueueCopy(SimulationManager* sim)
{
    SDL_LockMutex(drawingQueueMutex_);

    //Update vision sensor transforms and copy generated data to ensure consistency
    glMemoryBarrier(GL_PIXEL_BUFFER_BARRIER_BIT);
    for(size_t i=0; i < content_->GetViewsCount(); ++i)
        content_->GetView(i)->UpdateTransform();
    //Update light transforms to ensure consistency
    for(size_t i=0; i < content_->GetLightsCount(); ++i)
        content_->GetLight(i)->UpdateTransform();
    //Update ocean currents for particle systems
    Ocean* ocean = sim->GetOcean();
    if(ocean != nullptr) ocean->UpdateVelocityFieldsData();

    if(!drawingQueue_.empty())
    {
        drawingQueueCopy_.clear();
        selectedDrawingQueueCopy_.clear();
        //Double buffering
        drawingQueueCopy_.insert(drawingQueueCopy_.end(), drawingQueue_.begin(), drawingQueue_.end());
        selectedDrawingQueueCopy_.insert(selectedDrawingQueueCopy_.end(), selectedDrawingQueue_.begin(), selectedDrawingQueue_.end());
        //Enable update of drawing queue by clearing old queue
        drawingQueue_.clear(); 
        selectedDrawingQueue_.clear();
    }

    SDL_UnlockMutex(drawingQueueMutex_);

    //Sort objects by material to reduce uniform/texture switching
    std::sort(drawingQueueCopy_.begin(), drawingQueueCopy_.end(), Renderable::SortByMaterial);
}

void OpenGLPipeline::DrawDisplay()
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, screenFBO_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, rSettings_.windowW, rSettings_.windowH, 0, 0, rSettings_.windowW, rSettings_.windowH, GL_COLOR_BUFFER_BIT, GL_NEAREST);
}

void OpenGLPipeline::DrawObjects()
{
    for(size_t i=0; i<drawingQueueCopy_.size(); ++i)
    {
		if (drawingQueueCopy_[i].type == RenderableType::SOLID)
        {
			content_->DrawObject(drawingQueueCopy_[i].objectId, drawingQueueCopy_[i].lookId, drawingQueueCopy_[i].model);
        }
        else if(drawingQueueCopy_[i].type == RenderableType::CABLE)
        {
            auto nodes = drawingQueueCopy_[i].GetDataAsCableNodes();
            content_->DrawCable(drawingQueueCopy_[i].objectId, drawingQueueCopy_[i].model[0][0], *nodes, drawingQueueCopy_[i].lookId);
        }
    }
}

void OpenGLPipeline::DrawLights()
{
    for(size_t i=0; i<content_->GetLightsCount(); ++i)
        content_->DrawLightSource(i);
}
    
void OpenGLPipeline::DrawHelpers()
{
    //Coordinate systems
    if(hSettings_.showCoordSys)
    {
        content_->DrawCoordSystem(glm::mat4(1.f), 1.f);
        
        for(size_t h=0; h<drawingQueueCopy_.size(); ++h)
        {
            if(drawingQueueCopy_[h].type == RenderableType::SOLID_CS)
                content_->DrawCoordSystem(drawingQueueCopy_[h].model, 0.25f);
        }
    }
    
    //Discrete and multibody joints
    if(hSettings_.showJoints)
    {
        for(size_t h=0; h<drawingQueueCopy_.size(); ++h)
        {
            if(drawingQueueCopy_[h].type == RenderableType::MULTIBODY_AXIS)
                content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,0.5f,1.f,1.f), drawingQueueCopy_[h].model);
            else if(drawingQueueCopy_[h].type == RenderableType::JOINT_LINES)
                content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,0.5f,1.f,1.f), drawingQueueCopy_[h].model);
            else if(drawingQueueCopy_[h].type == RenderableType::PATH_POINTS)
                content_->DrawPrimitives(PrimitiveType::POINTS, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,0.5f,1.f,1.f), drawingQueueCopy_[h].model);
            else if(drawingQueueCopy_[h].type == RenderableType::PATH_LINE_STRIP)
                content_->DrawPrimitives(PrimitiveType::LINE_STRIP, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,0.5f,1.f,1.f), drawingQueueCopy_[h].model);
        }
    }
    
    //Sensors
    if(hSettings_.showSensors)
    {
        for(size_t h=0; h<drawingQueueCopy_.size(); ++h)
        {
            if(drawingQueueCopy_[h].type == RenderableType::SENSOR_CS)
                content_->DrawCoordSystem(drawingQueueCopy_[h].model, 0.25f);
            else if(drawingQueueCopy_[h].type == RenderableType::SENSOR_POINTS)
                content_->DrawPrimitives(PrimitiveType::POINTS, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,1.f,0,1.f), drawingQueueCopy_[h].model);
            else if(drawingQueueCopy_[h].type == RenderableType::SENSOR_LINES)
                content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,1.f,0,1.f), drawingQueueCopy_[h].model);
            else if(drawingQueueCopy_[h].type == RenderableType::SENSOR_LINE_STRIP)
                content_->DrawPrimitives(PrimitiveType::LINE_STRIP, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,1.f,0,1.f), drawingQueueCopy_[h].model);
        }
    }
    
    //Actuators
    if(hSettings_.showActuators)
    {
        for(size_t h=0; h<drawingQueueCopy_.size(); ++h)
        {
            if(drawingQueueCopy_[h].type == RenderableType::ACTUATOR_LINES)
                content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,0.5f,0,1.f), drawingQueueCopy_[h].model);
        }
    }
    
    //Fluid dynamics
    if(hSettings_.showFluidDynamics)
    {
        for(size_t h=0; h<drawingQueueCopy_.size(); ++h)
        {
            switch(drawingQueueCopy_[h].type)
            {
                case RenderableType::HYDRO_CS:
                    content_->DrawEllipsoid(drawingQueueCopy_[h].model, glm::vec3(0.005f), glm::vec4(0.3f, 0.7f, 1.f, 1.f));
                    break;
                    
                case RenderableType::HYDRO_CYLINDER:
                    content_->DrawCylinder(drawingQueueCopy_[h].model, drawingQueueCopy_[h].GetDataAsPoints().get()->at(0), glm::vec4(0.2f, 0.5f, 1.f, 1.f));
                    break;
                    
                case RenderableType::HYDRO_ELLIPSOID:
                    content_->DrawEllipsoid(drawingQueueCopy_[h].model, drawingQueueCopy_[h].GetDataAsPoints().get()->at(0), glm::vec4(0.2f, 0.5f, 1.f, 1.f));
                    break;
                    
                case RenderableType::HYDRO_POINTS:
                    content_->DrawPrimitives(PrimitiveType::POINTS, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(0.3f, 0.7f, 1.f, 1.f), drawingQueueCopy_[h].model);
                    break;
                    
                case RenderableType::HYDRO_LINES:
                    content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(0.2f, 0.5f, 1.f, 1.f), drawingQueueCopy_[h].model);
                    break;
                    
                case RenderableType::HYDRO_LINE_STRIP:
                    content_->DrawPrimitives(PrimitiveType::LINE_STRIP, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(0.2f, 0.5f, 1.f, 1.f), drawingQueueCopy_[h].model);
                    break;

                case RenderableType::HYDRO_TRIANGLES:
                    content_->DrawPrimitives(PrimitiveType::TRIANGLES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(0.2f, 0.5f, 1.f, 1.f), drawingQueueCopy_[h].model);
                    break;
                    
                default:
                    break;
            }
        }
    }
    
    //Forces
    if(hSettings_.showForces)
    {
        for(size_t h=0; h<drawingQueueCopy_.size(); ++h)
        {
            switch(drawingQueueCopy_[h].type)
            {
                case RenderableType::FORCE_BUOYANCY:
                    content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(0.f,0.f,1.f,1.f), drawingQueueCopy_[h].model);
                    break;
        
                case RenderableType::FORCE_LINEAR_DRAG:
                    content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(0.f,1.f,1.f,1.f), drawingQueueCopy_[h].model);
                    break;
                    
                case RenderableType::FORCE_QUADRATIC_DRAG:
                    content_->DrawPrimitives(PrimitiveType::LINES, drawingQueueCopy_[h].GetDataAsPoints().get(), glm::vec4(1.f,0.f,1.f,1.f), drawingQueueCopy_[h].model);
                    break;
        
                default:
                    break;
            }
        }
    }
}

bool OpenGLPipeline::Render(SimulationManager* sim, bool onlyIfUpdated)
{	
    //Update time step for animation purposes
    Scalar now = sim->GetSimulationTime();
    Scalar dt = now-lastSimTime_;
    lastSimTime_ = now;

    //Double-buffering of drawing queue
    PerformDrawingQueueCopy(sim);
	
    //Choose rendering mode
    unsigned int renderMode = 0; //Defaults to rendering without ocean
    Ocean* ocean = sim->GetOcean();
    Atmosphere* atm = sim->GetAtmosphere();
    if(ocean != nullptr)
    {
        ocean->GetOpenGlOcean()->Simulate(dt);
        causticsDt_ += dt;
        renderMode = rSettings_.ocean > RenderQuality::DISABLED && ocean->IsRenderable() ? 1 : 0;
    }
    //Update the queue of views needing update
    auto frameTime = std::chrono::steady_clock::now();
    viewQueued_.resize(content_->GetViewsCount());
    viewRendered_.resize(content_->GetViewsCount());
    viewPeriod_.resize(content_->GetViewsCount(), 0.0);
    unsigned int updateCount = 0;
    std::vector<unsigned int> viewsNoUpdate; //View that are not needing update but have to be displayed
    std::vector<unsigned int> pairedViews; //Views rendered together with the view they are paired with
    for(int i=content_->GetViewsCount()-1; i >= 0; --i) //Go through views in reverse order
    {
        OpenGLView* view = content_->GetView(i);
        
        if(!view->IsEnabled()) //Skip disabled views
            continue;

        if(view->GetType() == ViewType::SEGMENTATION_CAMERA 
           && static_cast<OpenGLSegmentationCamera*>(view)->GetPairedView() != nullptr)
        {
            pairedViews.push_back(i);
            continue;
        }
      
        if(view->NeedsUpdate())
        {
            if(view->IsContinuous()) //Has to always get updated independent from the number of views in the queue
            {
                if(std::find(viewsQueue_.begin(), viewsQueue_.end(), i) == viewsQueue_.end()) //Not already in the queue
                {
                    viewsQueue_.push_front(i);
                    ++updateCount;
                }
            }
            else // Will be updated now or in the following frames
            {
                if(std::find(viewsQueue_.begin(), viewsQueue_.end(), i) == viewsQueue_.end()) //Not already in the queue
                {
                    viewsQueue_.push_back(i);
                    viewsNoUpdate.push_back(i);
                    viewQueued_[i] = frameTime;
                }
            }
        }
        else // Does not need update
        {
            if(std::find(viewsNoUpdate.begin(), viewsNoUpdate.end(), i) == viewsNoUpdate.end()) //Not already in the queue
                viewsNoUpdate.push_back(i);
        }
    }

    //One sensor camera is rendered per frame, as they become due, but the fastest one sets the pace: the slower cameras
    //and the window are rendered right after its frames (in the next call, or in the first half of its period), so that
    //they do not delay its next one, unless they have waited for maxDelay. While cameras run the window is rendered at
    //most every windowInterval and at least every windowMaxAge; without them, at most every windowIntervalAlone. A minimized
    //window is not rendered.
    constexpr std::chrono::milliseconds maxDelay(60), windowInterval(50), windowIntervalAlone(33), windowMaxAge(200);
    double fastestPeriod = 0.0;
    for(size_t i=0; i<viewPeriod_.size(); ++i) //Of the cameras still running
        if(viewPeriod_[i] > 0.0 && frameTime - viewRendered_[i] < std::chrono::seconds(1)
           && (fastestPeriod == 0.0 || viewPeriod_[i] < fastestPeriod))
            fastestPeriod = viewPeriod_[i];
    auto isFast = [&](unsigned int id) { return fastestPeriod == 0.0 || viewPeriod_[id] <= 1.5 * fastestPeriod; };
    ++framesSinceFast_;
    bool inSlot = framesSinceFast_ == 1 || std::chrono::duration<double>(frameTime - lastFastFrame_).count() < 0.5 * fastestPeriod;

    if(viewsQueue_.size() > content_->GetViewsCount()) // Safety condition to avoid runaway
    {
        updateCount = viewsQueue_.size();
        viewsNoUpdate.clear();
    }
    else
    {
        auto next = std::find_if(viewsQueue_.begin() + updateCount, viewsQueue_.end(), 
            [&](unsigned int id) { return isFast(id) || inSlot || frameTime - viewQueued_[id] >= maxDelay; });
        if(next != viewsQueue_.end())
        {
            unsigned int id = *next;
            viewsQueue_.erase(next);
            viewsQueue_.insert(viewsQueue_.begin() + updateCount, id);
            ++updateCount;
            viewsNoUpdate.erase(std::remove(viewsNoUpdate.begin(), viewsNoUpdate.end(), id), viewsNoUpdate.end());
        }
    }

    bool sensorFrame = std::any_of(viewsQueue_.begin(), viewsQueue_.begin() + updateCount, 
        [this](unsigned int id) { return content_->GetView(id)->GetType() != ViewType::TRACKBALL; });
    bool camerasRunning = frameTime - lastSensorFrame_ < windowMaxAge;
    auto windowAge = frameTime - lastWindowFrame_;
    bool windowDue = !windowMinimized_ && (camerasRunning ? windowAge >= windowMaxAge || (!sensorFrame && inSlot && windowAge >= windowInterval)
                                                          : windowAge >= windowIntervalAlone);
    for(unsigned int i=0; i<updateCount;)
    {
        if(content_->GetView(viewsQueue_[i])->GetType() != ViewType::TRACKBALL)
            ++i;
        else if(!windowDue)
        {
            viewsNoUpdate.push_back(viewsQueue_[i]);
            viewsQueue_.erase(viewsQueue_.begin() + i);
            --updateCount;
        }
        else
        {
            lastWindowFrame_ = frameTime;
            ++i;
        }
    }
    if(sensorFrame)
        lastSensorFrame_ = frameTime;
    //The window's view fills the screen. When it is not rendered, the screen keeps its last image: its render buffers
    //may have been reused since (the selection outline draws in them), so it cannot be tone mapped again.
    bool windowKept = std::any_of(viewsNoUpdate.begin(), viewsNoUpdate.end(), 
        [this](unsigned int id) { return content_->GetView(id)->GetType() == ViewType::TRACKBALL; });
    for(unsigned int i=0; i<updateCount; ++i)
    {
        unsigned int id = viewsQueue_[i];
        if(content_->GetView(id)->GetType() == ViewType::TRACKBALL)
            continue;
        if(viewRendered_[id].time_since_epoch().count() > 0)
        {
            double dt = std::chrono::duration<double>(frameTime - viewRendered_[id]).count();
            viewPeriod_[id] = viewPeriod_[id] > 0.0 ? 0.9 * viewPeriod_[id] + 0.1 * dt : dt;
        }
        viewRendered_[id] = frameTime;
        if(isFast(id))
        {
            lastFastFrame_ = frameTime;
            framesSinceFast_ = 0;
        }
    }

    //Nothing new to show
    if(updateCount == 0 && onlyIfUpdated)
        return false;

    //Caustics, for the sensor cameras (the window shows the last ones, unless there are no cameras), at most 30 times
    //per second of simulation: cameras rendered in quick succession share them
    if(ocean != nullptr && updateCount > 0 && (sensorFrame || frameTime - lastSensorFrame_ > windowMaxAge)
       && causticsDt_ >= Scalar(1)/Scalar(30))
    {
        ocean->GetOpenGlOcean()->GenerateCaustics(atm->GetOpenGlAtmosphere()->GetSunDirection(), causticsDt_);
        causticsDt_ = Scalar(0);
    }

    OpenGLState::EnableDepthTest();
    OpenGLState::EnableCullFace();
    
    //Bake shadow maps for lights (independent of view)
    content_->SetupLights();
    if(rSettings_.shadows > RenderQuality::DISABLED)
    {
        glCullFace(GL_FRONT);
        glDisable(GL_DEPTH_CLAMP);
        content_->SetDrawingMode(DrawingMode::SHADOW);
        for(size_t i=0; i<content_->GetLightsCount(); ++i)
        {
            if(content_->GetLight(i)->IsActive())
                content_->GetLight(i)->BakeShadowmap(this);
        }
        glEnable(GL_DEPTH_CLAMP);
        glCullFace(GL_BACK);
    }
    
    //Clear display framebuffer
    OpenGLState::BindFramebuffer(screenFBO_);
    if(!windowKept)
        glClear(GL_COLOR_BUFFER_BIT);

    //Loop through all views -> trackballs, cameras, depth cameras...
    for(unsigned int i=0; i<updateCount; ++i)
    {  
        OpenGLState::EnableDepthTest();
        OpenGLState::EnableCullFace();
        OpenGLState::DisableBlend();
        OpenGLView* view = content_->GetView(viewsQueue_[i]);
    
        switch(view->GetType())
        { 
            case ViewType::DEPTH_CAMERA:
            {
                OpenGLDepthCamera* camera = static_cast<OpenGLDepthCamera*>(view);
                //Draw objects and compute depth data
                camera->ComputeOutput(drawingQueueCopy_);
                //Draw camera output
                camera->DrawLDR(screenFBO_, true);
            }
            break;

            case ViewType::THERMAL_CAMERA:
            {
                OpenGLThermalCamera* camera = static_cast<OpenGLThermalCamera*>(view);
                glm::vec3 eye = camera->GetEyePosition();

                if(renderMode == 1 && ocean->GetDepth(eye) > 0.f) //Camera underwater (not working)
                {
                    //Clear camera to water temperature
                    OpenGLState::BindFramebuffer(camera->GetRenderFbo());
                    glDrawBuffer(GL_COLOR_ATTACHMENT0);
                    glClearColor(ocean->GetOpenGlOcean()->GetWaterTemperature(), 0.f, 0.f, 0.f);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                }
                else //Camera above water or no ocean enabled
                {
                    //Apply view properties
                    std::vector<GLint> viewport = camera->GetViewport();
                    content_->SetViewportSize(viewport[2],viewport[3]);
                
                    //Bake parallel-split shadowmaps for sun
                    if(rSettings_.shadows > RenderQuality::DISABLED)
                    {
                        content_->SetDrawingMode(DrawingMode::SHADOW);
                        atm->GetOpenGlAtmosphere()->BakeShadowmaps(this, camera);
                    }
                    atm->GetOpenGlAtmosphere()->SetupMaterialShaders();
    
                    //Clear main framebuffer and setup camera
                    OpenGLState::BindFramebuffer(camera->GetRenderFbo());
                    glDrawBuffer(GL_COLOR_ATTACHMENT0);
                    glClearColor(0.f, 0.f, 0.f, 0.f);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    
                    camera->SetViewport();
                    content_->SetCurrentView(camera);
                    
                    //Draw scene
                    if(renderMode == 0) //NO OCEAN
                    {
                        //Render all objects
                        content_->SetDrawingMode(DrawingMode::TEMPERATURE);
                        DrawObjects();
                    
                        //Render sky (at the end to take profit of early bailing)
                        atm->GetOpenGlAtmosphere()->DrawSkyAndSunTemperature(camera);
                    }
                    else if(renderMode == 1) //OCEAN
                    {
                        OpenGLOcean* glOcean = ocean->GetOpenGlOcean();
                        
                        //Update ocean for this camera
                        if(ocean->HasWaves())
                            glOcean->UpdateSurface(camera);

                        //Rendering only above water
                        //possible because camera near plane is (virtually) removed with logarithmic depth buffer.
                        glOcean->DrawSurfaceTemperature(camera);
                        //Draw all objects as above surface 
                        //(depth testing will secure drawing only what is above water)
                        content_->SetDrawingMode(DrawingMode::TEMPERATURE);
                        DrawObjects();
                        
                        //Render sky (left for the end to only fill empty spaces)
                        atm->GetOpenGlAtmosphere()->DrawSkyAndSunTemperature(camera);
                    }
                }
                //Flip thermal image and render display texture
                camera->ComputeOutput();

                //Draw camera output
                camera->DrawLDR(screenFBO_, true);
            }
            break;

            case ViewType::OPTICAL_FLOW_CAMERA:
            {
                OpenGLOpticalFlowCamera* camera = static_cast<OpenGLOpticalFlowCamera*>(view);
                //Draw objects and compute camera data
                camera->ComputeOutput(drawingQueueCopy_);
                //Draw camera output
                camera->DrawLDR(screenFBO_, true);
            }
            break;

            case ViewType::SEGMENTATION_CAMERA:
            {
                OpenGLSegmentationCamera* camera = static_cast<OpenGLSegmentationCamera*>(view);
                //Draw objects and compute camera data
                camera->ComputeOutput(drawingQueueCopy_, ocean);
                //Draw camera output
                camera->DrawLDR(screenFBO_, true);
            }
            break;
            
            case ViewType::SONAR:
            {
                OpenGLSonar* sonar = static_cast<OpenGLSonar*>(view);
                //Draw objects and compute sonar data
                sonar->ComputeOutput(drawingQueueCopy_);
                //Draw sonar output
                sonar->DrawLDR(screenFBO_, true);
            }
            break;

            case ViewType::CAMERA:
            case ViewType::TRACKBALL:
            case ViewType::EVENT_BASED_CAMERA:
            {
                //Apply view properties
                OpenGLCamera* camera = static_cast<OpenGLCamera*>(view);
                OpenGLLight::SetCamera(camera);
                std::vector<GLint> viewport = camera->GetViewport();
                content_->SetViewportSize(viewport[2],viewport[3]);
                bool underwater = renderMode == 1 && ocean->GetDepth(camera->GetEyePosition()) > 0.0;
            
                //Bake parallel-split shadowmaps for sun (refracted by the water surface when looking from under water)
                if(rSettings_.shadows > RenderQuality::DISABLED)
                {
                    content_->SetDrawingMode(DrawingMode::SHADOW);
                    atm->GetOpenGlAtmosphere()->BakeShadowmaps(this, camera, underwater);
                }
                atm->GetOpenGlAtmosphere()->SetupMaterialShaders();
            
                //Clear main framebuffer and setup camera
                OpenGLState::BindFramebuffer(camera->GetRenderFbo());
                camera->SetRenderBuffers(0, true, false);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
                
                camera->SetViewport();
                content_->SetCurrentView(camera);
                
                //Draw scene
                if(renderMode == 0) //NO OCEAN
                {
                    //Render all objects
                    content_->SetDrawingMode(DrawingMode::FULL);
                    DrawObjects();
                    DrawLights();

                    //Ambient occlusion
                    if(rSettings_.ao > RenderQuality::DISABLED)
                        camera->DrawAO(1.0f);
                    
                    //Render sky (at the end to take profit of early bailing)
                    atm->GetOpenGlAtmosphere()->DrawSkyAndSun(camera);
                }
                else if(renderMode == 1) //OCEAN
                {
                    OpenGLOcean* glOcean = ocean->GetOpenGlOcean();
                    
                    //Update ocean for this camera
                    if(ocean->HasWaves())
                        glOcean->UpdateSurface(camera);

                    //Two separate rendering paths: above water and under water, 
                    //possible because camera near plane is (virtually) removed with logarithmic depth buffer.
                    if(underwater)
                    {  
                        content_->SetDrawingMode(DrawingMode::UNDERWATER);
                        DrawObjects();
                        if(!ocean->HasWaves() && camera->CanSeeWaterSurface()) //Mirror image of the scene on the flat water surface (total internal reflection)
                        {
                            //Distorted by the ripples, it does without the sun's shadows, which cost most of its rendering
                            atm->GetOpenGlAtmosphere()->SetupMaterialShaders(false);
                            camera->RenderWaterReflection(this);
                            atm->GetOpenGlAtmosphere()->SetupMaterialShaders();
                        }
                        glOcean->DrawBackground(camera);
                        glOcean->DrawBacksurface(camera);
                        //camera->GenerateBloom();
                        DrawLights();
                        
                        //Without waves the mirror image reflects the scene in the surface, so only reflective looks need SSR
                        if(rSettings_.ssr > RenderQuality::DISABLED && (ocean->HasWaves() || content_->HasReflectiveLooks()))
                        {
                            //Linear depth front faces
                            camera->GenerateLinearDepth(true);
                            
                            //Linear depth back faces
                            OpenGLState::BindFramebuffer(camera->GetPostprocessFbo());
                            glClear(GL_DEPTH_BUFFER_BIT);
                            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                            glCullFace(GL_FRONT);
                            content_->SetDrawingMode(DrawingMode::FLAT);
                            DrawObjects();
                            glCullFace(GL_BACK);
                            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                            camera->GenerateLinearDepth(false);
                            
                            //Draw screen-space reflections (including total internal reflection at the water surface)
                            camera->DrawSSR(true);
                        }

                        //Draw bloom effect simulating scattering
                        //camera->DrawBloom((GLfloat)ocean->getWaterType());

                        //Suspended particles only below surface
                        glDepthMask(GL_FALSE);
                        glOcean->DrawParticles(camera);
                        glDepthMask(GL_TRUE);
                        
                    }
                    else //Above water
                    {
                        content_->SetDrawingMode(DrawingMode::UNDERWATER);
                        DrawObjects();
                        DrawLights();
                        glOcean->DrawBackground(camera);

                        //Draw surface to back buffer
                        camera->SetRenderBuffers(1, false, true); //Clearing color buffer
                        camera->SetRenderBuffers(1, true, false); //Color + Normal
                        glOcean->DrawSurface(camera);
                        camera->SetRenderBuffers(0, false, false); //Color only
                        
                        //Blend surface on top of scene
                        OpenGLState::DisableDepthTest();
                        OpenGLState::EnableBlend();
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        content_->DrawTexturedSAQ(camera->GetColorTexture(1));
                        OpenGLState::DisableBlend();
                        OpenGLState::EnableDepthTest();
                        
                        //Draw all objects as above surface 
                        //(depth testing will secure drawing only what is above water)
                        camera->SetRenderBuffers(0, true, false); //Color + Normal
                        content_->SetDrawingMode(DrawingMode::FULL);
                        DrawObjects();
                        DrawLights();
                    
                        //Render sky (left for the end to only fill empty spaces)
                        atm->GetOpenGlAtmosphere()->DrawSkyAndSun(camera);    

                        //Postprocess (the window does without the reflections in the surface, a third of its rendering)
                        if(rSettings_.ssr > RenderQuality::DISABLED && camera->GetType() != ViewType::TRACKBALL)
                        {
                            //Linear depth front faces
                            camera->GenerateLinearDepth(true);
                        
                            //Linear depth back faces
                            OpenGLState::BindFramebuffer(camera->GetPostprocessFbo());
                            glClear(GL_DEPTH_BUFFER_BIT);
                            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                            glCullFace(GL_FRONT);
                            content_->SetDrawingMode(DrawingMode::FLAT);
                            DrawObjects();
                            glCullFace(GL_BACK);
                            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                            camera->GenerateLinearDepth(false);
                            
                            //Draw screen-space reflections
                            camera->DrawSSR();
                        }
                    }
                }

                //Special case for event-based cameras
                if(camera->GetType() == ViewType::EVENT_BASED_CAMERA)
                    static_cast<OpenGLEventBasedCamera*>(camera)->ComputeOutput(now);

                //Drawing to the screen
                camera->DrawLDR(screenFBO_, true);
            
                //Non-reallistic rendering (graphical information)
                if(camera->GetType() == ViewType::TRACKBALL)
                {
                    //Overlay debugging info
                    OpenGLState::BindFramebuffer(screenFBO_); //No depth buffer, just one color buffer
                    content_->SetProjectionMatrix(camera->GetProjectionMatrix());
                    content_->SetViewMatrix(camera->GetViewMatrix());
                    OpenGLState::DisableCullFace();
                    
                    //Simulation debugging
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    //if(sim->getSolidDisplayMode() == DisplayMode::PHYSICAL) DrawObjects();
                    DrawHelpers();
                    if(hSettings_.showOceanVelocityField && ocean != NULL) ocean->GetOpenGlOcean()->DrawVelocityField(camera, 5.f);
                    if(hSettings_.showBulletDebugInfo) sim->RenderBulletDebug();
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                    
                    //Overlay selection outline
                    ((OpenGLTrackball*)camera)->DrawSelection(selectedDrawingQueueCopy_, screenFBO_);
                    
                    //Graphics debugging
                    //if(ocean != NULL)
                    //{
                        //ocean->getOpenGLOcean()->ShowSpectrum(glm::vec2(512.f,512.f), glm::vec4(0.f,0.f,512.f,512.f));
                        //ocean->getOpenGLOcean()->ShowTexture(3, glm::vec4(0,0,512,512));
                    //}
            
                    //atm->getOpenGLAtmosphere()->ShowSunShadowmaps(0, 0, 0.1f);
                    
                    //content->DrawTexturedQuad(0,0,800,600,camera->getPostprocessTexture(0));
                    //camera->ShowViewNormalTexture(glm::vec4(0,0,400,300));
                    //camera->ShowSceneTexture(glm::vec4(0,200,300,200));
                    //camera->ShowLinearDepthTexture(glm::vec4(0,0,600,500), true);
                    //camera->ShowLinearDepthTexture(glm::vec4(0,400,300,200), false);
                    
                    //camera->ShowDeinterleavedDepthTexture(glm::vec4(0,400,300,200), 0);
                    //camera->ShowDeinterleavedDepthTexture(glm::vec4(0,400,300,200), 8);
                    //camera->ShowDeinterleavedDepthTexture(glm::vec4(0,600,300,200), 9);
                    //camera->ShowDeinterleavedAOTexture(glm::vec4(0,600,300,200), 0);
                    //camera->ShowAmbientOcclusion(glm::vec4(0,800,300,200));
                    //camera->ShowDepthStencilTexture(glm::vec4(0,400,300,200)); 
                                    
                    OpenGLState::BindFramebuffer(0);
                }
            }
            break;
        }
    }
    //Segmentation cameras paired with a view rendered in this frame, when they are due: they render the same scene
    //from the same viewpoint, so that they label its image exactly
    for(size_t i=0; i<pairedViews.size(); ++i)
    {
        OpenGLSegmentationCamera* camera = static_cast<OpenGLSegmentationCamera*>(content_->GetView(pairedViews[i]));
        auto rendered = std::find_if(viewsQueue_.begin(), viewsQueue_.begin() + updateCount, 
            [this, camera](unsigned int id) { return content_->GetView(id) == camera->GetPairedView(); });
        if(rendered != viewsQueue_.begin() + updateCount && camera->NeedsUpdate())
        {
            OpenGLState::EnableDepthTest();
            OpenGLState::EnableCullFace();
            OpenGLState::DisableBlend();
            camera->ComputeOutput(drawingQueueCopy_, ocean);
            camera->DrawLDR(screenFBO_, true);
        }
        else
            viewsNoUpdate.push_back(pairedViews[i]);
    }

    //Draw views that are displayed but not updated (the window's last image is still on the screen)
    for(size_t i=0; i<viewsNoUpdate.size(); ++i)
    {
        OpenGLView* view = content_->GetView(viewsNoUpdate[i]);
        if(view->GetType() != ViewType::TRACKBALL)
            view->DrawLDR(screenFBO_, false);
    }
    //Remove views drawn in this frame
    viewsQueue_.erase(viewsQueue_.begin(), viewsQueue_.begin() + updateCount);
    return true;
}

}

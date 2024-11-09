#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include <iostream>
#include <vector>

#include "Engine/Formats.hpp"
#include "Engine/app.h"
#include "Engine/TextureND.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/UI.h"
#include "Labs/4-Animation/CasePBD.h"
#include "imgui.h"
namespace VCX::Labs::Animation
{
    CasePBD::CasePBD():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _particlesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points),
        _springsItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    bool CasePBD::is_active(MassSpringSystem &system,int i)
    {
        
            return !system.Fixed[i];
       
    }
    void CasePBD::simulationStep(MassSpringSystem & system, float dt) {
            int   num = 8;
            float ddt           = dt / num;
            for (int s = 0; s < num; s++) {
                float alpha = 0.0;
                if (system.Stiffness != 0.0)
                    alpha = 1.0 / system.Stiffness;
                float     alpha_tilde = alpha / (ddt * ddt);
                std::vector<glm::vec3> pOld(system.Positions.size());
                /*glm::vec3 pOld(0.0f);*/
                for (int i = 0; i < system.Positions.size(); i++) {
                    if (is_active(system, i)) {
                        glm::vec3 & p = system.Positions[i];
                        glm::vec3 & v = system.Velocities[i];

                        pOld[i]=p;

                        system.Velocities[i].y = v.y - ddt * system.Gravity;

                        system.Positions[i].x = p.x + ddt * v.x;
                        system.Positions[i].y = p.y + ddt * v.y;
                        system.Positions[i].z = p.z + ddt * v.z;
                        // p.z remains the same
                    }
                }

                for (auto & spring : system.Springs)
                    spring.lambda = 0.0;

                for (int iter = 0; iter < system.numIterations; iter++) {
                    for (auto & spring : system.Springs) {
                        glm::vec3 & p1 = system.Positions[spring.AdjIdx.first];
                        glm::vec3 & p2 = system.Positions[spring.AdjIdx.second];

                        glm::vec3 dp = p1 - p2;
                        
                        float dl     = glm::length(dp);

                        float C = dl - spring.RestLength;

                        glm::vec3 gradC(0.0f);
                        if (dl > 0.00001) {
                            gradC.x   = dp.x / dl;
                            gradC.y = dp.y / dl;
                            gradC.z   =dp.z / dl;
                            float K = glm::dot(gradC, gradC) / system.Mass;
                            if (spring.AdjIdx.first != 0)
                                K = 2 * K;
                            K += alpha_tilde;

                            float delta_lambda = -(C + alpha_tilde * spring.lambda) / K;
                            spring.lambda += delta_lambda;

                            if (is_active(system, spring.AdjIdx.first)) {
                                system.Positions[spring.AdjIdx.first].x = p1.x + (1.0f / system.Mass) * delta_lambda * gradC.x;
                                system.Positions[spring.AdjIdx.first].y = p1.y + (1.0f / system.Mass) * delta_lambda * gradC.y;
                                system.Positions[spring.AdjIdx.first].z = p1.z + (1.0f / system.Mass) * delta_lambda * gradC.z;
                            }
                            if (is_active(system, spring.AdjIdx.second)) {
                                system.Positions[spring.AdjIdx.second].x = p2.x - (1.0f / system.Mass) * delta_lambda * gradC.x;
                                system.Positions[spring.AdjIdx.second].y = p2.y - (1.0f / system.Mass) * delta_lambda * gradC.y;
                                system.Positions[spring.AdjIdx.second].z = p2.z - (1.0f / system.Mass) * delta_lambda * gradC.z;
                                // p2.z remains the same
                            }
                        }
                    }
                }

                for (int i = 0; i < system.Positions.size(); i++) {
                    if (is_active(system, i)) {
                        glm::vec3 & p = system.Positions[i];
                        glm::vec3 & v = system.Velocities[i];

                        system.Velocities[i].x = (1.0f / ddt) * (p.x - pOld[i].x);
                        system.Velocities[i].y = (1.0f / ddt) * (p.y - pOld[i].y);
                        system.Velocities[i].z = (1.0f / ddt) * (p.z - pOld[i].z);
                        // v.z remains the same
                    }
                }
            }
    }
    void CasePBD::ResetSystem()
    {
            _massSpringSystem       = {};
            std::size_t const n     = 20;
            float const       delta = 2.f / n;
            auto constexpr GetID    = [](std::size_t const i, std::size_t const j) { return i * (n + 1) + j; };
            for (std::size_t i = 0; i <= n; i++) {
                for (std::size_t j = 0; j <= n; j++) {
                    _massSpringSystem.AddParticle(glm::vec3(i * delta, 1.5f, j * delta - 1.f));
                    if (i > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j));
                    if (i > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 2, j));
                    if (j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 1));
                    if (j > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 2));
                    if (i > 0 && j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j - 1));
                    if (i > 0 && j < n) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j + 1));
                }
            }
            if (fixedparticles==1) {
                _massSpringSystem.Fixed[GetID(0, 0)] = true;
                
            } else if (fixedparticles==2) {
                _massSpringSystem.Fixed[GetID(0, 0)] = true;
                _massSpringSystem.Fixed[GetID(0, n)] = true;
               
            }
            else
            {
                _massSpringSystem.Fixed[GetID(0, 0)] = true;
                _massSpringSystem.Fixed[GetID(0, n)] = true;
                _massSpringSystem.Fixed[GetID(n, 0)] = true;
                _massSpringSystem.Fixed[GetID(n, n)] = true;
            }
            
            std::vector<std::uint32_t> indices;
            for (auto const & spring : _massSpringSystem.Springs) {
                indices.push_back(std::uint32_t(spring.AdjIdx.first));
                indices.push_back(std::uint32_t(spring.AdjIdx.second));
            }
            _springsItem.UpdateElementBuffer(indices);
    }
    
    void CasePBD::OnSetupPropsUI()
    {
            if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
                static const char * items[]      = { "1", "2", "4" };
                static int          current_item = 0; // 默认选择第一个

                // 创建一个ListBox控件
                if (ImGui::ListBox("fixedparticle", &current_item, items, IM_ARRAYSIZE(items))) {
                    
                    fixedparticles = atoi(items[current_item]);
                    ResetSystem();
                }
                        
                
                if (ImGui::Button("Reset System")) ResetSystem();
                ImGui::SameLine();
                if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
                ImGui::SliderFloat("Part. Mass", &_massSpringSystem.Mass, .5f, 10.f);
                ImGui::SliderFloat("Spr. Stiff.", &_massSpringSystem.Stiffness, 10.f, 300.f);
                ImGui::SliderFloat("Spr. Damp.", &_massSpringSystem.Damping, .1f, 10.f);
                ImGui::SliderFloat("Gravity", &_massSpringSystem.Gravity, .1f, 1.f);
            }
            ImGui::Spacing();

            if (ImGui::CollapsingHeader("Appearance")) {
                ImGui::SliderFloat("Part. Size", &_particleSize, 1, 6);
                ImGui::ColorEdit3("Part. Color", glm::value_ptr(_particleColor));
                ImGui::SliderFloat("Spr. Width", &_springWidth, .001f, 1.f);
                ImGui::ColorEdit3("Spr. Color", glm::value_ptr(_springColor));
            }
            ImGui::Spacing();
    }
    Common::CaseRenderResult CasePBD::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize)
    {
            if (! _stopped) simulationStep(_massSpringSystem, Engine::GetDeltaTime());

            _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));
            _springsItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));

            _frame.Resize(desiredSize);

            _cameraManager.Update(_camera);

            _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

            gl_using(_frame);
            glEnable(GL_LINE_SMOOTH);
            glPointSize(_particleSize);
            glLineWidth(_springWidth);

            _program.GetUniforms().SetByName("u_Color", _springColor);
            _springsItem.Draw({ _program.Use() });
            _program.GetUniforms().SetByName("u_Color", _particleColor);
            _particlesItem.Draw({ _program.Use() });

            glLineWidth(1.f);
            glPointSize(1.f);
            glDisable(GL_LINE_SMOOTH);

            return Common::CaseRenderResult {
                .Fixed     = false,
                .Flipped   = true,
                .Image     = _frame.GetColorAttachment(),
                .ImageSize = desiredSize,
            };
    }
    void CasePBD::OnProcessInput(ImVec2 const & pos)
    {
            _cameraManager.ProcessInput(_camera, pos);
    }
}
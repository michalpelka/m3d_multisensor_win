#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "ImGuizmo/ImGuizmo.h"
#include "structs.h"

std::vector<Sophus::Vector6f> extrinsic_calib_vec;
std::vector<calib_struct::plane> calibration_planes;
struct calibration_plane {
    Eigen::Matrix4f unmuttable_transform;
    Eigen::Vector4d plane_params;
    Eigen::Vector2d plane_size{ 1,1 };
};
glm::vec2 clicked_point;
float rot_x = 0.0f;
float rot_y = 0.0f;
bool drawing_buffer_dirty = true;
glm::vec3 view_translation{ 0,0,-30 };



void cursor_calback(GLFWwindow* window, double xpos, double ypos)
{
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureMouse) {
        const glm::vec2 p{ -xpos, ypos };
        const auto d = clicked_point - p;
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
            rot_x += 0.01 * d[1];
            rot_y += 0.01 * d[0];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) {
            view_translation[2] += 0.02 * d[1];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_3) == GLFW_PRESS) {
            view_translation[1] += 0.01 * d[1];
            view_translation[0] -= 0.01 * d[0];
        }
        clicked_point = p;
    }

}


void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

int main(int argc, char** argv) {
    calibration_planes.resize(1);
    std::string fn1 = argv[1];
    std::string fn2 = argv[2];

    //extrinsic_calib_vec = { Sophus::Vector6f::Zero(),Sophus::Vector6f::Zero() };
    extrinsic_calib_vec = calib_struct::initializeCalib();
    GLFWwindow* window;
    const char* glsl_version = "#version 130";
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(960, 540, "rgbd_demo", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, cursor_calback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { return -1; }

    GLCall(glClearColor(0.4, 0.4, 0.4, 1));

    Renderer renderer;

    VertexBufferLayout layout;
    layout.Push<float>(3);
    layout.Push<float>(3);

    VertexArray va_co;
    VertexBuffer vb_co(gl_primitives::coordinate_system_vertex.data(),
        gl_primitives::coordinate_system_vertex.size() * sizeof(float));
    va_co.AddBuffer(vb_co, layout);
    IndexBuffer ib_co(gl_primitives::coordinate_system_indices.data(), gl_primitives::coordinate_system_indices.size());

    VertexArray va_plane;
    VertexBuffer vb_plane(gl_primitives::plane_vertex.data(),
        gl_primitives::plane_vertex.size() * sizeof(float));
    va_plane.AddBuffer(vb_plane, layout);
    IndexBuffer ib_plane(gl_primitives::plane_idices.data(), gl_primitives::plane_idices.size());

    Shader shader(shader_simple_v, shader_simple_f);
    Shader shader_pc(shader_pc_intensity_v, shader_pc_intensity_f);
    Shader shader_pc_head(shader_pc_intensity_head_v, shader_pc_intensity_f);

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn1,*cloud2);
    int len2 = 0; 
    std::shared_ptr<float[]> data2 = calib_struct::pclToBuffer(cloud2, len2, 1.0f);
    std::shared_ptr<calib_struct::KeyFrame> k2 = std::make_shared<calib_struct::KeyFrame>(data2, len2, Eigen::Matrix4d::Identity(),2);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn2, *cloud1);
    int len1 = 0;
    std::shared_ptr<float[]> data1 = calib_struct::pclToBuffer(cloud1, len1, 1.0f);
    std::shared_ptr<calib_struct::KeyFrame> k1 = std::make_shared<calib_struct::KeyFrame>(data1, len1, Eigen::Matrix4d::Identity(), 1);

    
    int plane_edited = 0;
    while (!glfwWindowShouldClose(window)) {
        GLCall(glEnable(GL_DEPTH_TEST));
        GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuiIO& io = ImGui::GetIO();
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        glm::mat4 proj = glm::perspective(30.f, 1.0f * width / height, 0.05f, 1000.0f);
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(0.1f, 0.1f, 0.1f));
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_1 = glm::rotate(model_translate, rot_x, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(0.0f, 0.0f, 1.0f));
        glm::mat4 model_rotation_3 = glm::rotate(model_rotation_2, (float)(0.5f * M_PI), glm::vec3(-1.0f, 0.0f, 0.0f));

        if (plane_edited > -1)
        {
            Eigen::Matrix4f gizmo_mat{ calibration_planes[plane_edited].getGizmo()}; 
            ImGuizmo::Manipulate(&model_rotation_2[0][0], &proj[0][0], ImGuizmo::TRANSLATE_Z | ImGuizmo::TRANSLATE_Y | ImGuizmo::TRANSLATE_X | ImGuizmo::ROTATE_Y,
                ImGuizmo::LOCAL, gizmo_mat.data());
            calibration_planes[plane_edited].setGizmo(gizmo_mat);
            shader.Bind(); // bind shader to apply uniform

            for (const auto& p : calibration_planes)
            {
                glm::mat4 mat;
                Eigen::Map<Eigen::Matrix4f> map_mat(&mat[0][0]);
                map_mat = p.matrix;
                glm::mat4 plane_rot = glm::rotate(glm::mat4(1.0f), (float)(0.5f * M_PI), glm::vec3(1.0f, 0.0f, 0.0f));
                shader.setUniformMat4f("u_MVP", proj * model_rotation_2 * mat);
                renderer.Draw(va_co, ib_co, shader, GL_LINES);

                shader.setUniformMat4f("u_MVP", proj * model_rotation_2 * mat * glm::scale(glm::mat4(1.0f), glm::vec3(p.size.x(), p.size.y(), 1.0f)));
                renderer.Draw(va_plane, ib_plane, shader, GL_TRIANGLES);
            }
        }

        glm::mat4 uniform_head_matrix;
        Eigen::Map<Eigen::Matrix4f> map_uniform_head_matrix(&uniform_head_matrix[0][0]);
        map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[0]).matrix();
        //map_uniform_head_matrix.block<3, 3>(0, 0) = Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitX()).toRotationMatrix();
        shader_pc_head.Bind();
        shader_pc_head.setUniformMat4f("u_MVPPC", proj * model_rotation_2 );
        shader_pc_head.setUniformMat4f("u_HEAD", uniform_head_matrix);
        shader_pc_head.setUniform4f("u_COLORPC", 1, 1, 0, 1);
        shader_pc_head.setUniform1f("u_AngOffset", 0);
        renderer.DrawArray(k2->va, shader_pc_head, GL_POINTS, k2->len / 6);


        map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[1]).matrix();
        //map_uniform_head_matrix.block<3, 3>(0, 0) = (Eigen::AngleAxisf((-14.5*M_PI/180.0), Eigen::Vector3f::UnitY())*
        //    Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitX())).toRotationMatrix();
        shader_pc_head.setUniformMat4f("u_MVPPC", proj * model_rotation_2);
        shader_pc_head.setUniformMat4f("u_HEAD", uniform_head_matrix);
        shader_pc_head.setUniform4f("u_COLORPC", 1, 0, 0, 1);
        shader_pc_head.setUniform1f("u_AngOffset", 0);
        renderer.DrawArray(k1->va, shader_pc_head, GL_POINTS, k1->len / 6);


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ImGui::Begin("Calibration Demo");
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        for (int i = 0; i < extrinsic_calib_vec.size(); i++) {
            ImGui::Text("Calib %d:", i);
            ImGui::DragFloat3(("calib_r" + std::to_string(i)).c_str(), extrinsic_calib_vec[i].data(), 0.001f);
            ImGui::DragFloat3(("calib_t" + std::to_string(i)).c_str(), extrinsic_calib_vec[i].data()+3, 0.0001f);
        }
        if (ImGui::Button("Reset")) {
            extrinsic_calib_vec = calib_struct::initializeCalib();
        }
        ImGui::SameLine();
        if (ImGui::Button("Load")) {
            extrinsic_calib_vec = calib_struct::initializeCalib("calib.txt");
        }
        ImGui::SameLine();
        if (ImGui::Button("Save")) {
            calib_struct::saveConfig(extrinsic_calib_vec, "calib.txt");
        }
        if (ImGui::Button("ExportPDC")) {
            auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[1]);
            auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[0]);
            pcl::io::savePCDFileBinary("test1_pcd.pcd", pcd1);
            pcl::io::savePCDFileBinary("test2_pcd.pcd", pcd2);
        }

        if (ImGui::Button("LoadPlanes")) {
            calibration_planes = calib_struct::loadPlanes("planes.txt");
        }
        ImGui::SameLine();
        if (ImGui::Button("SavePlanes")) {
            calib_struct::savePlanes(calibration_planes, "planes.txt");
        }
        if (ImGui::Button("ExportPDC")) {
            auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[1]);
            auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[0]);
            pcl::io::savePCDFileBinary("test1_pcd.pcd", pcd1);
            pcl::io::savePCDFileBinary("test2_pcd.pcd", pcd2);
        }
        for (int i = 0; i < calibration_planes.size(); i++)
        {
            char b[256];
            snprintf(b, 256, "edit_plane%d", i);
            if (ImGui::Button(b)) {
                plane_edited = i;
            }
            ImGui::SameLine();
            snprintf(b, 256, "rem_plane%d", i);
            if (ImGui::Button(b)) {
                calibration_planes.erase(calibration_planes.begin() + i);
                break;
            }
            ImGui::SameLine();
            snprintf(b, 256, "hv%d", i);
            float* f = calibration_planes[i].size.data();
            ImGui::InputFloat2(b,f);
            ImGui::SameLine();
            snprintf(b, 256, "export plane %d", i);
            if (ImGui::Button(b)) {

                auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[1]);
                auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[0]);
                std::vector <pcl::PointCloud<pcl::PointXYZI>> pcds{ pcd1,pcd2 };

                pcl::PointCloud<pcl::PointXYZL> cloud;
                
                auto plane_se3 = calibration_planes[i].matrix;
                auto plane_size = calibration_planes[i].size;

                auto plane_se3_inv = plane_se3.inverse();
                Eigen::Vector3f plane_vec_inv = plane_se3_inv.block<1, 3>(1, 3);
                for (int j = 0; j < pcds.size(); j++)
                {
                    auto& pcd = pcds[j];
                    cloud.reserve(cloud.size() + pcd.size());
                    for (int i = 0; i < pcd.size(); i++) {
                        //Eigen::Vector3f p = pcd[i].getArray3fMap() - plane_vec_inv;
                        pcl::PointXYZL pi;
                        pi.label = j;
                        Eigen::Vector4f p = pcd[i].getArray4fMap();
                        pi.getArray4fMap() = plane_se3_inv * p ;
                        if (abs(pi.x) < plane_size.x() && abs(pi.y) < plane_size.y() && abs(pi.z) < 1.0) {
                            cloud.push_back(pi);
                        }

                    }
                }
                snprintf(b, 256, "cloud_plane%d.pcd", i);
                pcl::io::savePCDFileBinary(b, cloud);
            }
            snprintf(b, 256, "svd %d", i);
            ImGui::SameLine();
            if (ImGui::Button(b)) {


                auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[1]);
                auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[0]);
                std::vector <pcl::PointCloud<pcl::PointXYZI>> pcds{ pcd1,pcd2 };

                std::vector<Eigen::Vector4f> cloud;

                auto plane_se3 = calibration_planes[i].matrix;
                auto plane_size = calibration_planes[i].size;

                auto plane_se3_inv = plane_se3.inverse();
                
                Eigen::Vector3f plane_vec_inv = plane_se3_inv.block<1, 3>(1, 3);
                for (int j = 0; j < pcds.size(); j++)
                {
                    auto& pcd = pcds[j];
                    cloud.reserve(cloud.size() + pcd.size());
                    for (int i = 0; i < pcd.size(); i++) {
                        Eigen::Vector4f p = pcd[i].getArray4fMap();
                        Eigen::Vector4f pi = plane_se3_inv * p;
                        if (abs(pi.x()) < plane_size.x() && abs(pi.y()) < plane_size.y() && abs(pi.z()) < 1.0) {
                            cloud.push_back(p);
                        }
                    }
                }
                const Eigen::Vector4f avg_center = calib_struct::avgT<Eigen::Vector4f>(cloud);
                Eigen::Matrix3f covariance = calib_struct::findCovariance(cloud, avg_center);
                Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullV);

                calibration_planes[i].matrix = Eigen::Matrix4f::Identity();
                Eigen::Vector3f z = svd.matrixV().col(2);
                calibration_planes[i].matrix.block<3, 3>(0, 0).col(2) = z;
                calibration_planes[i].matrix.block<3, 3>(0, 0).col(1) = z.cross(Eigen::Vector3f{ 1,0,0 });
                calibration_planes[i].matrix.block<3, 3>(0, 0).col(0) = z.cross(Eigen::Vector3f{ 0,0,1 });

                calibration_planes[i].matrix.block<4, 1>(0, 3) = avg_center;
            }

        }
        if (ImGui::Button("add_plane")) {
            calibration_planes.resize(calibration_planes.size() + 1);
        }

        ImGui::End();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;

}
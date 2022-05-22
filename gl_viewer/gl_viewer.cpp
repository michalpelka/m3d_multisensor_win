#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <boost/program_options.hpp>
#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "ImGuizmo/ImGuizmo.h"
#include "gl_calib_livox/structs.h"
std::vector<Sophus::Vector6f> extrinsic_calib_vec;
const std::vector<float> encoder_offset{0,0,M_PI};
std::array<bool,3> locked_params{false,false,true};

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
    namespace po = boost::program_options;
    po::options_description description("MyTool Usage");

    description.add_options()
            ("help,h", "Display this help message")
            ("input_data,i", po::value<std::string>(),"points to input data")
            ("calibration_file,c",po::value<std::string>()->default_value("calibs.txt"), "calibration file");
    std::cout << description << "\n";
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    po::notify(vm);



    if (vm.count("help")) {
        std::cout << description << "\n";
        return 1;
    }

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
    Shader shader_pc_head_spherical_img(shader_pc_spherical_head_v, shader_pc_spherical_f);


    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    const std::string fn = vm["input_data"].as<std::string>();
    const std::string calibration_file = vm["calibration_file"].as<std::string>();
    std::cout << "*********************************" << std::endl;
    std::cout << "        fn        : " << fn << std::endl;
    std::cout << " calibration_file : " << calibration_file << std::endl;

    extrinsic_calib_vec = calib_struct::initializeCalib(calibration_file);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn+"_pointcloud_raw_livox_1.pcd",*cloud2);
    int len2 = 0;

    std::shared_ptr<float[]> data2 = calib_struct::pclToBuffer(cloud2, len2, 1.0f);
    std::shared_ptr<calib_struct::KeyFrame> k2 = std::make_shared<calib_struct::KeyFrame>(data2, len2, Eigen::Matrix4d::Identity(),2);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn+"_pointcloud_raw_livox_2.pcd", *cloud1);

    int len1 = 0;
    std::shared_ptr<float[]> data1 = calib_struct::pclToBuffer(cloud1, len1, 1.0f);
    std::shared_ptr<calib_struct::KeyFrame> k1 = std::make_shared<calib_struct::KeyFrame>(data1, len1, Eigen::Matrix4d::Identity(), 1);
    int shader_program_no = 0;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_velo(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn+"_pointcloud_raw_velodyne.pcd", *cloud_velo);

    int len3 = 0;
    std::shared_ptr<float[]> data3 = calib_struct::pclToBuffer(cloud_velo, len3, 1.0f);
    std::shared_ptr<calib_struct::KeyFrame> k_velo = std::make_shared<calib_struct::KeyFrame>(data3, len3, Eigen::Matrix4d::Identity(), 1);

    const std::vector< pcl::PointCloud<pcl::PointXYZINormal>::Ptr> raw_pointclouds{cloud1,cloud2, cloud_velo};

    int plane_edited = 0;
    Texture tex (fn+"_ladybugPostProcessing-panoramic-0-radius_100.000000.jpg");
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


        glm::mat4 uniform_head_matrix;
        glm::mat4 uniform_ladybug_matrix;

        Eigen::Map<Eigen::Matrix4f> map_uniform_head_matrix(&uniform_head_matrix[0][0]);
        Eigen::Map<Eigen::Matrix4f> map_uniform_ladybug_matrix(&uniform_ladybug_matrix[0][0]);

        //map_uniform_head_matrix.block<3, 3>(0, 0) = Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitX()).toRotationMatrix();
        if (shader_program_no == 0) {
            shader_pc_head.Bind();
            map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[0]).matrix();
            shader_pc_head.setUniformMat4f("u_MVPPC", proj * model_rotation_2);
            shader_pc_head.setUniformMat4f("u_HEAD", uniform_head_matrix);
            shader_pc_head.setUniform4f("u_COLORPC", 1, 1, 0, 1);
            shader_pc_head.setUniform1f("u_AngOffset", encoder_offset[0]);
            renderer.DrawArray(k1->va, shader_pc_head, GL_POINTS, k2->len / 6);

            map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[1]).matrix();
            shader_pc_head.setUniformMat4f("u_MVPPC", proj * model_rotation_2);
            shader_pc_head.setUniformMat4f("u_HEAD", uniform_head_matrix);
            shader_pc_head.setUniform4f("u_COLORPC", 1, 0, 0, 1);
            shader_pc_head.setUniform1f("u_AngOffset", encoder_offset[1]);
            renderer.DrawArray(k2->va, shader_pc_head, GL_POINTS, k1->len / 6);

            map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[2]).matrix();
            shader_pc_head.setUniformMat4f("u_MVPPC", proj * model_rotation_2);
            shader_pc_head.setUniformMat4f("u_HEAD", uniform_head_matrix);
            shader_pc_head.setUniform4f("u_COLORPC", 1, 0, 1, 1);
            shader_pc_head.setUniform1f("u_AngOffset",encoder_offset[2]);
            renderer.DrawArray(k_velo->va, shader_pc_head, GL_POINTS, k_velo->len / 6);


        }
        if (shader_program_no == 1) {
            shader_pc_head_spherical_img.Bind();
            tex.Bind(1);
            shader_pc_head_spherical_img.setUniform1i("u_Texture", 1);
            map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[0]).matrix();
            map_uniform_ladybug_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[3]).matrix();
            shader_pc_head_spherical_img.setUniformMat4f("u_MVPPC", proj * model_rotation_2);
            shader_pc_head_spherical_img.setUniformMat4f("u_HEAD", uniform_head_matrix);
            shader_pc_head_spherical_img.setUniformMat4f("u_LADYBUG", uniform_ladybug_matrix);

            shader_pc_head_spherical_img.setUniform4f("u_COLORPC", 1, 1, 0, 1);
            shader_pc_head_spherical_img.setUniform1f("u_AngOffset", 0);
            renderer.DrawArray(k1->va, shader_pc_head_spherical_img, GL_POINTS, k2->len / 6);

            map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[1]).matrix();
            shader_pc_head_spherical_img.setUniformMat4f("u_MVPPC", proj * model_rotation_2);
            shader_pc_head_spherical_img.setUniformMat4f("u_HEAD", uniform_head_matrix);
            shader_pc_head_spherical_img.setUniform4f("u_COLORPC", 1, 0, 0, 1);
            shader_pc_head_spherical_img.setUniform1f("u_AngOffset", 0);
            renderer.DrawArray(k2->va, shader_pc_head_spherical_img, GL_POINTS, k1->len / 6);

            map_uniform_head_matrix = Sophus::SE3f::exp(extrinsic_calib_vec[2]).matrix();
            shader_pc_head_spherical_img.setUniformMat4f("u_MVPPC", proj * model_rotation_2);
            shader_pc_head_spherical_img.setUniformMat4f("u_HEAD", uniform_head_matrix);
            shader_pc_head_spherical_img.setUniform4f("u_COLORPC", 1, 0, 1, 1);
            shader_pc_head_spherical_img.setUniform1f("u_AngOffset",encoder_offset[2]);
            renderer.DrawArray(k_velo->va, shader_pc_head_spherical_img, GL_POINTS, k_velo->len / 6);

        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ImGui::Begin("Calibration Demo");
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        //ImGui::Image((ImTextureID)tex.getMRendererId(), ImVec2(0.1*tex.GetWidth(),0.1*tex.GetHeight()));
        const char* items[] = { "ShaderIntensity", "ShaderSpherical", "ShaderPerspetive"};
        ImGui::Combo("Shader", &shader_program_no,items,3);


        if (ImGui::Button("ExportPDC")) {
            auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[0],encoder_offset[0]);
            auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[1],encoder_offset[1]);
            auto pcd3 = calib_struct::createTransformedPc(cloud_velo, extrinsic_calib_vec[2],encoder_offset[2]);

            pcl::io::savePCDFileBinary("test1_pcd.pcd", pcd1);
            pcl::io::savePCDFileBinary("test2_pcd.pcd", pcd2);
            pcl::io::savePCDFileBinary("test3_pcd.pcd", pcd3);

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
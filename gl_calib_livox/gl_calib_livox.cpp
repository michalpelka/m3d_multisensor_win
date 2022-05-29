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
#include "structs.h"
#include <ceres/ceres.h>
#include "cost_fun.h"

std::vector<Sophus::Vector6f> extrinsic_calib_vec;
std::vector<calib_struct::plane> calibration_planes;
const std::vector<float> encoder_offset{0,0,M_PI};
std::array<bool,3> locked_params{false,false,true};

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


template <typename T> Eigen::Matrix<T,4,1>getPlaneCoefFromSE3(const Eigen::Matrix<T,4,4>& SE3){
    const T a = -SE3(0,2);
    const T b = -SE3(1,2);
    const T c = -SE3(2,2);
    const T d = SE3(0,3) * a + SE3(1,3) * b + SE3(2,3) * c;
    return Eigen::Matrix<T,4,1> {a,b,c,d};
}

void cursor_calback(GLFWwindow* window, double xpos, double ypos)
{
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureMouse) {
        const glm::vec2 p{ -xpos, ypos };
        const auto d = clicked_point - p;
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
            rot_x += 0.05 * d[1];
            rot_y += 0.05 * d[0];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) {
            view_translation[2] += 0.02 * d[1];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_3) == GLFW_PRESS) {
            view_translation[1] += 0.05 * d[1];
            view_translation[0] -= 0.05 * d[0];
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
            ("calibration_planes,p",po::value<std::string>()->default_value("planes.txt"), "calibration planes file")
            ("calibration_file,c",po::value<std::string>()->default_value("calibs.txt"), "calibration file");
    std::cout << description << "\n";
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    po::notify(vm);


    if (vm.count("help")) {
        std::cout << description << "\n";
        return 1;
    }
    calibration_planes.resize(1);
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
    Shader shader_pc_head_spherical_img(shader_pc_spherical_head_v, shader_pc_spherical_f);


    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    //std::string fn = "/media/michal/ext/test_tele15_velo/1653040409";
    //std::string fn = "/media/michal/ext/test_tele15_velo/1653040329";
    //std::string fn = "/media/michal/ext/data_root/1650712239";
    //std::string fn = "/media/michal/ext/data/1652552048";
//    const std::string fn = "/media/michal/ext/15MAJ2022_Skierniewice_test_1_husky+2xtele15/1652612566";
    const std::string fn = vm["input_data"].as<std::string>();
    const std::string plane_file = vm["calibration_planes"].as<std::string>();
    const std::string calibration_file = vm["calibration_file"].as<std::string>();
    std::cout << "*********************************" << std::endl;
    std::cout << "        fn        : " << fn << std::endl;
    std::cout << " plane_file       : " << plane_file << std::endl;
    std::cout << " calibration_file : " << calibration_file << std::endl;

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

        if (plane_edited > -1)
        {
            Eigen::Matrix4f gizmo_mat{ calibration_planes[plane_edited].getGizmo()};
            ImGuizmo::Manipulate(&model_rotation_2[0][0], &proj[0][0], ImGuizmo::TRANSLATE_Z | ImGuizmo::TRANSLATE_Y | ImGuizmo::TRANSLATE_X | ImGuizmo::ROTATE_Y | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X,
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

                shader.setUniformMat4f("u_MVP", proj * model_rotation_2 * mat * glm::scale(glm::mat4(1.0f), glm::vec3 (p.size.x(), p.size.y(), 1.0f)));
                renderer.Draw(va_plane, ib_plane, shader, GL_TRIANGLES);
            }
        }

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

        for (int i = 0; i < extrinsic_calib_vec.size(); i++) {
            ImGui::Text("Calib %d:", i);
            ImGui::SameLine();
            if (i <locked_params.size()) ImGui::Checkbox(("block_" + std::to_string(i)).c_str(),&locked_params[i]);
            ImGui::DragFloat3(("calib_r" + std::to_string(i)).c_str(), extrinsic_calib_vec[i].data(), 0.001f);
            ImGui::DragFloat3(("calib_t" + std::to_string(i)).c_str(), extrinsic_calib_vec[i].data()+3, 0.0001f);
        }
        if (ImGui::Button("Reset")) {
            extrinsic_calib_vec = calib_struct::initializeCalib();
        }
        ImGui::SameLine();
        if (ImGui::Button("Load")) {
            extrinsic_calib_vec = calib_struct::initializeCalib(calibration_file);
        }
        ImGui::SameLine();
        if (ImGui::Button("Save")) {
            calib_struct::saveConfig(extrinsic_calib_vec, calibration_file);
        }
        if (ImGui::Button("ExportPDC")) {
            auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[0],encoder_offset[0]);
            auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[1],encoder_offset[1]);
            auto pcd3 = calib_struct::createTransformedPc(cloud_velo, extrinsic_calib_vec[2],encoder_offset[2]);

            pcl::io::savePCDFileBinary("test1_pcd.pcd", pcd1);
            pcl::io::savePCDFileBinary("test2_pcd.pcd", pcd2);
            pcl::io::savePCDFileBinary("test3_pcd.pcd", pcd3);

        }

        if (ImGui::Button("LoadPlanes")) {
            calibration_planes = calib_struct::loadPlanes(plane_file);
        }
        ImGui::SameLine();
        if (ImGui::Button("SavePlanes")) {
            calib_struct::savePlanes(calibration_planes, plane_file);
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

                auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[0],encoder_offset[0]);
                auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[1],encoder_offset[1]);
                auto pcd3 = calib_struct::createTransformedPc(cloud_velo, extrinsic_calib_vec[2],encoder_offset[2]);

                std::vector <pcl::PointCloud<pcl::PointXYZINormal>> pcds{ pcd1,pcd2, pcd3 };
                pcl::io::savePCDFileBinary("test1_pcd.pcd", pcd1);
                pcl::io::savePCDFileBinary("test2_pcd.pcd", pcd2);
                pcl::io::savePCDFileBinary("test2_pcd.pcd", pcd3);

                auto plane_se3 = calibration_planes[i].matrix;
                auto plane_size = calibration_planes[i].size;
                auto plane_se3_inv = plane_se3.inverse();

                Eigen::Vector3f plane_vec_inv = plane_se3_inv.block<1, 3>(1, 3);
                for (int j = 0; j < pcds.size(); j++)
                {
                    auto& pcd = pcds[j];
                    pcl::PointCloud<pcl::PointXYZL> cloud;
                    cloud.reserve(cloud.size() + pcd.size());
                    for (int i = 0; i < pcd.size(); i++) {
                        //Eigen::Vector3f p = pcd[i].getArray3fMap() - plane_vec_inv;
                        pcl::PointXYZL pi;
                        pi.label = j;
                        Eigen::Vector4f p = pcd[i].getArray4fMap();
                        pi.getArray4fMap() = plane_se3_inv * p ;
                        if (abs(pi.x) < plane_size.x() && abs(pi.y) < plane_size.y() && abs(pi.z) < 2.0) {
                            cloud.push_back(pi);
                        }

                    }
                    snprintf(b, 256, "cloud_plane%d_cloud%d.pcd", i,j);
                    pcl::io::savePCDFileBinary(b, cloud);
                }

            }
            snprintf(b, 256, "svd %d", i);
            ImGui::SameLine();
            if (ImGui::Button(b)) {

                std::vector <pcl::PointCloud<pcl::PointXYZINormal>> pcds{ };

                for (int i =0; i <locked_params.size(); i++){
                    if (locked_params[i]) {
                        pcds.emplace_back(
                                calib_struct::createTransformedPc(raw_pointclouds[i], extrinsic_calib_vec[i], encoder_offset[i]));
                    }
                }

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
                        if (abs(pi.x()) < plane_size.x() && abs(pi.y()) < plane_size.y() && abs(pi.z()) < 1) {
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
        if (ImGui::Button("calib")){
            ceres::Problem problem;
            std::vector<Eigen::Vector4d> plane_params;
            std::vector<Sophus::SE3d> laser_params;

            // params
            for (int i =0; i < calibration_planes.size(); i++ ){
                //Eigen::Matrix4d m = calibration_planes[i].matrix.cast<double>();
                //plane_params.push_back(Sophus::SE3d::fitToSE3(m.inverse()));
                Eigen::Vector4d c = calibration_planes[i].getABCD();
                plane_params.push_back(c);
            }
            for (int i =0; i < calibration_planes.size(); i++ ) {
                problem.AddParameterBlock(plane_params[i].data(), 4, new LocalParameterizationPlane());
                //problem.SetParameterBlockConstant(plane_params[i].data());
            }

            for (int i =0; i < locked_params.size(); i++ ) {
                auto sp = Sophus::SE3f::exp(extrinsic_calib_vec[i]);
                std::cout << "sp :  " << sp.matrix() << std::endl;
                laser_params.push_back(sp.cast<double>());
            }
            for (int i =0; i < laser_params.size(); i++ ) {
                problem.AddParameterBlock(laser_params[i].data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3());
                if (locked_params[i]){
                    problem.SetParameterBlockConstant(laser_params[i].data());
                }
                ceres::CostFunction *cost_function = LockTranslation::Create();
                problem.AddResidualBlock(cost_function, nullptr, laser_params[i].data());
            }
            auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[0], encoder_offset[0]).makeShared();
            auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[1], encoder_offset[1]).makeShared();
            auto pcd3 = calib_struct::createTransformedPc(cloud_velo, extrinsic_calib_vec[2], encoder_offset[2]).makeShared();
            std::vector <pcl::PointCloud<pcl::PointXYZINormal>::Ptr> pcds{ pcd1,pcd2, pcd3 };
            std::vector <pcl::PointCloud<pcl::PointXYZINormal>::Ptr> raw_pcds{ cloud1,cloud2,cloud_velo };
            // residuals
            double res = 0;
            for (int i = 0; i < pcds.size(); i++)
            {

                const auto &pcd = *pcds[i];
                const auto raw_pcd = raw_pcds[i];

                for (int j =0; j < plane_params.size(); j++){

                    auto plane_size = calibration_planes[j].size;
                    auto plane_se3_inv =  calibration_planes[j].matrix.inverse().cast<double>();
                    Eigen::Vector3d plane_vec_inv = plane_se3_inv.block<1, 3>(1, 3);

                    for (int k =0; k < pcd.size(); k+=10){
                        Eigen::Vector4f p_float = pcd[k].getArray4fMap();
                        Eigen::Vector4d pi = plane_se3_inv * p_float.cast<double>();

                        if (abs(pi.x()) < plane_size.x() && abs(pi.y()) < plane_size.y() && abs(pi.z()) < 1.0) {

                            auto ppi = raw_pcd->at(k);
                            ppi.getVector4fMap() = pi.cast<float>();
                            //pc.push_back(ppi);

                            double angle =raw_pcd->at(k).normal_x;
                            ceres::LossFunction *loss = new ceres::CauchyLoss(0.1);
                            Eigen::Vector4f raw_point = raw_pcd->at(k).getArray4fMap();
                            ceres::CostFunction *cost_function =
                                    LaserSE3AgainstPlane::Create(raw_point.cast<double>(),angle+encoder_offset[i]);

                            LaserSE3AgainstPlane l(raw_point.cast<double>(),angle);
                            problem.AddResidualBlock(cost_function, loss, plane_params[j].data(), laser_params[i].data());
                            //std::exit(1);
                        }
                    }
                }
            }
            std::cout << "res :  " << res << std::endl;
//            pcl::io::savePCDFile("/tmp/pcd.pcd", pc);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 50;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
            for (int i = 0; i < pcds.size(); i++){
                std::cout <<"Update " << (laser_params[i].log().cast<float>() - extrinsic_calib_vec[i]).transpose() << std::endl;
                extrinsic_calib_vec[i] = laser_params[i].log().cast<float>();

            }
            for (int i = 0; i < calibration_planes.size(); i++){
                calibration_planes[i].setABCD(plane_params[i].cast<float>());
            }

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
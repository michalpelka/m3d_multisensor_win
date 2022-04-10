#include "GL/glwrapper.h"
#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>


std::vector<Sophus::Vector6f> extrinsic_calib_vec;

glm::vec2 clicked_point;
float rot_x = 0.0f;
float rot_y = 0.0f;
bool drawing_buffer_dirty = true;
glm::vec3 view_translation{ 0,0,-30 };

void saveConfig(std::vector<Sophus::Vector6f>& extrinsic_calib_vec, const std::string &fn) {
    std::ofstream txt(fn);
    for (const auto& p : extrinsic_calib_vec) {
        txt << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " " << p[4] << " " << p[5] << std::endl;
    }
    txt.close();
}

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

std::shared_ptr<float[]> pclToBuffer(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int& len, float scale)
{
    const int stride = 6;
    len = stride * cloud->size();
    std::shared_ptr<float[]> data(new float[len]);
    for (int i = 0; i < cloud->size(); i++) {
        data[stride * i + 0] = scale * (*cloud)[i].x;
        data[stride * i + 1] = scale * (*cloud)[i].y;
        data[stride * i + 2] = scale * (*cloud)[i].z;
        data[stride * i + 3] = (*cloud)[i].normal_x;
        data[stride * i + 4] = (*cloud)[i].normal_y;
        data[stride * i + 5] = (*cloud)[i].intensity;
    }
    return data;
}

struct KeyFrame {

    KeyFrame(std::shared_ptr<float[]> data_p, int len, Eigen::Matrix4d mat, int laser_id) :
        mat(mat), data(data_p), len(len), laser_id(laser_id),
        vb(data.get(), len * sizeof(float)),
        va()
    {
        timestamp = data[4];
        VertexBufferLayout layoutPc;
        layoutPc.Push<float>(3);
        layoutPc.Push<float>(1); // angle
        layoutPc.Push<float>(1); // ts
        layoutPc.Push<float>(1); // intensity
        va.AddBuffer(vb, layoutPc);
    }

    Eigen::Matrix4d mat;
    double timestamp;
    const std::shared_ptr<float[]> data;
    const int len;
    const int laser_id;
    VertexBuffer vb;
    VertexArray va;
};

std::vector<Sophus::Vector6f> initializeCalib() {
    Eigen::Matrix4f calibration1 = Eigen::Matrix4f::Identity();
    calibration1.block<3, 3>(0, 0) = Eigen::AngleAxisf(-M_PI / 2.0, Eigen::Vector3f::UnitX()).toRotationMatrix();

    Eigen::Matrix4f calibration2 = Eigen::Matrix4f::Identity();
    calibration2.block<3, 3>(0, 0) = (Eigen::AngleAxisf((-14.5 * M_PI / 180.0), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitX())).toRotationMatrix();

    return { Sophus::SE3f::fitToSE3(calibration1).log(),Sophus::SE3f::fitToSE3(calibration2).log(), };
}

std::vector<Sophus::Vector6f> initializeCalib(const std::string& fn) {
    std::ifstream txt(fn);
    std::string line;
    std::vector<Sophus::Vector6f> r;
    while (std::getline(txt, line))
    {
        std::stringstream oss(line);
        Sophus::Vector6f v;
        for (int i = 0; i < 6; i++)
        {
            oss >> v[i];
        }
        r.emplace_back(v);
    };
    txt.close();
    return r;
}

pcl::PointCloud<pcl::PointXYZI> createTransformedPc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& raw, Sophus::Vector6f& calib)
{
    pcl::PointCloud<pcl::PointXYZI> pc;
    const Eigen::Matrix4f calib_mat{ Sophus::SE3f::exp(calib).matrix() };
    pc.resize(raw->size());
    for (int i = 0; i < raw->size(); i++)
    {
        const auto raw_point = (*raw)[i];
        const float angle = raw_point.normal_x;
        const float s = sin(angle);
        const float c = cos(angle);
        Eigen::Matrix4f rot_angle;
        rot_angle <<c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        const Eigen::Vector4f p{ raw_point.getArray4fMap() };
        pc[i].getArray4fMap() = rot_angle.transpose() * calib_mat * p;
        pc[i].intensity = raw_point.intensity;
    }
    return pc;
}

int main(int argc, char** argv) {
    //extrinsic_calib_vec = { Sophus::Vector6f::Zero(),Sophus::Vector6f::Zero() };
    extrinsic_calib_vec = initializeCalib();
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

    Shader shader(shader_simple_v, shader_simple_f);
    Shader shader_pc(shader_pc_intensity_v, shader_pc_intensity_f);
    Shader shader_pc_head(shader_pc_intensity_head_v, shader_pc_intensity_f);

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>("D:/1646169749_pointcloud_raw_livox_1.pcd",*cloud2);
    int len2 = 0;
    std::shared_ptr<float[]> data2 = pclToBuffer(cloud2, len2, 1.0f);
    std::shared_ptr<KeyFrame> k2 = std::make_shared<KeyFrame>(data2, len2, Eigen::Matrix4d::Identity(),2);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>("D:/1646169749_pointcloud_raw_livox_2.pcd", *cloud1);
    int len1 = 0;
    std::shared_ptr<float[]> data1 = pclToBuffer(cloud1, len1, 1.0f);
    std::shared_ptr<KeyFrame> k1 = std::make_shared<KeyFrame>(data1, len1, Eigen::Matrix4d::Identity(), 1);


    while (!glfwWindowShouldClose(window)) {
        GLCall(glEnable(GL_DEPTH_TEST));
        GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiIO& io = ImGui::GetIO();
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glm::mat4 proj = glm::perspective(30.f, 1.0f * width / height, 0.05f, 1000.0f);
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(0.1f, 0.1f, 0.1f));
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_1 = glm::rotate(model_translate, rot_x, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(0.0f, 0.0f, 1.0f));
        glm::mat4 model_rotation_3 = glm::rotate(model_rotation_2, (float)(0.5f * M_PI), glm::vec3(-1.0f, 0.0f, 0.0f));

        glm::mat4 scan2_cfg;
        shader.Bind(); // bind shader to apply uniform

        shader.setUniformMat4f("u_MVP", proj * model_rotation_2);
        renderer.Draw(va_co, ib_co, shader, GL_LINES);

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
            extrinsic_calib_vec = initializeCalib();
        }
        ImGui::SameLine();
        if (ImGui::Button("Load")) {
            extrinsic_calib_vec = initializeCalib("calib.txt");
        }
        ImGui::SameLine();
        if (ImGui::Button("Save")) {
            saveConfig(extrinsic_calib_vec, "calib.txt");
        }
        if (ImGui::Button("ExportPDC")) {
            auto pcd1 = createTransformedPc(cloud1, extrinsic_calib_vec[1]);
            auto pcd2 = createTransformedPc(cloud2, extrinsic_calib_vec[0]);
            pcl::io::savePCDFileBinary("test1_pcd.pcd", pcd1);
            pcl::io::savePCDFileBinary("test2_pcd.pcd", pcd2);


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
#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "GL/glwrapper.h"
#include "ImGuizmo/ImGuizmo.h"

#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sophus/se3.hpp>
#include <ceres/ceres.h>

#include <opencv2/opencv.hpp>
#include "cost_fun.h"
#include "utils.h"
glm::vec2 clicked_point;
float rot_x =0.0f;
float rot_y =0.0f;
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

struct imgui_vars{
    float laser_config[6]= {0.13,0.f,0.f,0.f,0.f,0.f};
    float minimum_intesity{235};
    float max_distance{0.25};

    bool operator==(const imgui_vars &rhs) const {
        if(minimum_intesity != rhs.minimum_intesity) return false;
        if(max_distance != rhs.max_distance) return false;
        for (int i=0;i<6; i++){
            if(laser_config[i]  != rhs.laser_config[i] )return false;
        }
        return true;
    }
};
imgui_vars imgui_vars_curr;
imgui_vars imgui_vars_old;


void getCentroids(const pcl::PointCloud<pcl::PointXYZINormal> &pointcloud,
                  const Eigen::Affine3f &laser_local_calib,
                  pcl::PointCloud<pcl::PointXYZ>& centroid_left,
                  pcl::PointCloud<pcl::PointXYZ>& centroid_right,
                  std::vector<pcl::PointCloud<pcl::PointXYZINormal>>& centroid_left_raw,
                  std::vector<pcl::PointCloud<pcl::PointXYZINormal>>& centroid_right_raw,
                  int minimum_intensity = 200,
                  float max_distance = 0.1)
{

    pcl::PointCloud<pcl::PointXYZ> left;
    pcl::PointCloud<pcl::PointXYZ> right;
    pcl::PointCloud<pcl::PointXYZINormal> left_raw;
    pcl::PointCloud<pcl::PointXYZINormal> right_raw;

    Eigen::Affine3f calib(Eigen::Affine3f::Identity());
    calib.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));

    for (int i = 0; i < pointcloud.size(); i++) {
        const auto &p = pointcloud[i];
        if (p.intensity>minimum_intensity)
        {
            Eigen::Affine3f encoder_rot(Eigen::Affine3f::Identity());
            const float angle = -p.normal_x;
            Eigen::Affine3f transform(bigUnitCalib::getSE3OfLaser<float>(laser_local_calib.matrix(),angle));
            const auto pt = transform * p.getVector3fMap();
            pcl::PointXYZ pxyz;
            pxyz.getVector3fMap() = pt;
            if (pt.x()>0) {
                if (p.y > 0) {
                    left.push_back(pxyz);
                    left_raw.push_back(p);
                } else {
                    right.push_back(pxyz);
                    right_raw.push_back(p);
                }
            }

        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> group_left;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> group_right;

    // collapse left group
    for (const auto& p : left)
    {
        bool collapsed = false;
        for (auto& px : group_left)
        {
            const float dist = (p.getVector3fMap()-px.front().getVector3fMap()).norm();
            if (dist < 0.15)
            {
                collapsed = true;
                px.push_back(p);
            }
        }
        if (!collapsed)
        {
            pcl::PointCloud<pcl::PointXYZ> pp;
            pp.push_back(p);
            group_left.push_back(pp);
        }
    }

    // collapse left right
    for (const auto& p : right)
    {
        bool collapsed = false;
        for (auto& px : group_right)
        {
            const float dist = (p.getVector3fMap()-px.front().getVector3fMap()).norm();
            if (dist < 0.15)
            {
                collapsed = true;
                px.push_back(p);
            }
        }
        if (!collapsed)
        {
            pcl::PointCloud<pcl::PointXYZ> pp;
            pp.push_back(p);
            group_right.push_back(pp);
        }
    }
//    std::cout <<"group_left.size()" << group_left.size() << std::endl;
//    std::cout <<"group_right.size()" << group_right.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> centroid_left_temp;
    pcl::PointCloud<pcl::PointXYZ> centroid_right_temp;

    centroid_left.clear();
    centroid_right.clear();

    // compute per group centroid - left
    for (const auto& px : group_left) {
        pcl::PointXYZ centroid;
        centroid.getVector3fMap() = Eigen::Vector3f::Zero();
        for (const auto &p : px){
            centroid.getVector3fMap()+= p.getVector3fMap();
        }
        centroid.getVector3fMap()/=px.size();
        centroid_left_temp.push_back(centroid);
    }

    // compute per group centroid - right
    for (const auto& px : group_right) {
        pcl::PointXYZ centroid;
        centroid.getVector3fMap() = Eigen::Vector3f::Zero();
        for (const auto &p : px){
            centroid.getVector3fMap()+= p.getVector3fMap();
        }
        centroid.getVector3fMap()/=px.size();
        centroid_right_temp.push_back(centroid);
    }

    // assositation
    for (const auto &pr : centroid_right_temp)
    {
        double min_distance = std::numeric_limits<double>::max();
        pcl::PointXYZ best;
        for (const auto &pl : centroid_left_temp){
            double distance = (pl.getVector3fMap()-pr.getVector3fMap()).norm();
            if (distance < min_distance ){
                best = pl;
                min_distance = distance;
            }
        }
        if (min_distance < max_distance) {
            centroid_right.push_back(pr);
            centroid_left.push_back(best);
        }
    }


    // let's find closest raw mesurments to found centroids
    centroid_left_raw.clear();
    centroid_right_raw.clear();
    centroid_left_raw.resize(centroid_left.size());
    centroid_right_raw.resize(centroid_right.size());

    // left
    for (int j =0; j< centroid_left.size();j++)
    {
        const auto &pl = centroid_left[j];
        double min_distance = max_distance*0.1;
        ASSERT(left.size() == left_raw.size());
        for (int i =0; i < left.size(); i++)
        {
            float distance = (pl.getVector3fMap() - left[i].getVector3fMap()).norm();
            if (distance  < min_distance){
                centroid_left_raw[j].push_back(left_raw[i]);
            }
        }
    }
    //right
    for (int j =0; j< centroid_right.size();j++)
    {
        const auto &pr = centroid_right[j];
        double min_distance = max_distance*0.1;
        ASSERT(right.size() == right_raw.size());
        for (int i =0; i < right.size(); i++)
        {
            float distance = (pr.getVector3fMap() - right[i].getVector3fMap()).norm();
            if (distance < min_distance ){
                centroid_right_raw[j].push_back(right_raw[i]);
            }
        }
    }
}


void getCentroids(const pcl::PointCloud<pcl::PointXYZINormal> &pointcloud,
                  const Eigen::Affine3f &laser_local_calib,
                  pcl::PointCloud<pcl::PointXYZ>& centroid_left,
                  pcl::PointCloud<pcl::PointXYZ>& centroid_right,
                  pcl::PointCloud<pcl::PointXYZINormal>& centroid_left_raw,
                  pcl::PointCloud<pcl::PointXYZINormal>& centroid_right_raw,
                  int minimum_intensity = 200,
                  float max_distance = 0.1)
{

    pcl::PointCloud<pcl::PointXYZ> left;
    pcl::PointCloud<pcl::PointXYZ> right;
    pcl::PointCloud<pcl::PointXYZINormal> left_raw;
    pcl::PointCloud<pcl::PointXYZINormal> right_raw;


    Eigen::Affine3f calib(Eigen::Affine3f::Identity());
    calib.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));

    for (int i = 0; i < pointcloud.size(); i++) {
        const auto &p = pointcloud[i];
        if (p.intensity>minimum_intensity)
        {
            Eigen::Affine3f encoder_rot(Eigen::Affine3f::Identity());
            const float angle = -p.normal_x;
            Eigen::Affine3f transform(bigUnitCalib::getSE3OfLaser<float>(laser_local_calib.matrix(),angle));
            const auto pt = transform * p.getVector3fMap();
            pcl::PointXYZ pxyz;
            pxyz.getVector3fMap() = pt;
            if (pt.x()>0)
            {
                if (p.y > 0) {
                    left.push_back(pxyz);
                    left_raw.push_back(p);
                } else {
                    right.push_back(pxyz);
                    right_raw.push_back(p);
                }
            }
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> group_left;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> group_right;

    // collapse left group
    for (const auto& p : left)
    {
        bool collapsed = false;
        for (auto& px : group_left)
        {
            const float dist = (p.getVector3fMap()-px.front().getVector3fMap()).norm();
            if (dist < 0.15)
            {
                collapsed = true;
                px.push_back(p);
            }
        }
        if (!collapsed)
        {
            pcl::PointCloud<pcl::PointXYZ> pp;
            pp.push_back(p);
            group_left.push_back(pp);
        }
    }

    // collapse left right
    for (const auto& p : right)
    {
        bool collapsed = false;
        for (auto& px : group_right)
        {
            const float dist = (p.getVector3fMap()-px.front().getVector3fMap()).norm();
            if (dist < 0.15)
            {
                collapsed = true;
                px.push_back(p);
            }
        }
        if (!collapsed)
        {
            pcl::PointCloud<pcl::PointXYZ> pp;
            pp.push_back(p);
            group_right.push_back(pp);
        }
    }
    std::cout <<"group_left.size()" << group_left.size() << std::endl;
    std::cout <<"group_right.size()" << group_right.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> centroid_left_temp;
    pcl::PointCloud<pcl::PointXYZ> centroid_right_temp;

    centroid_left.clear();
    centroid_right.clear();

    // compute per group centroid - left
    for (const auto& px : group_left) {
        pcl::PointXYZ centroid;
        centroid.getVector3fMap() = Eigen::Vector3f::Zero();
        for (const auto &p : px){
            centroid.getVector3fMap()+= p.getVector3fMap();
        }
        centroid.getVector3fMap()/=px.size();
        centroid_left_temp.push_back(centroid);
    }

    // compute per group centroid - right
    for (const auto& px : group_right) {
        pcl::PointXYZ centroid;
        centroid.getVector3fMap() = Eigen::Vector3f::Zero();
        for (const auto &p : px){
            centroid.getVector3fMap()+= p.getVector3fMap();
        }
        centroid.getVector3fMap()/=px.size();
        centroid_right_temp.push_back(centroid);
    }

    // assositation
    for (const auto &pr : centroid_right_temp)
    {
        double min_distance = std::numeric_limits<double>::max();
        pcl::PointXYZ best;
        for (const auto &pl : centroid_left_temp){
            double distance = (pl.getVector3fMap()-pr.getVector3fMap()).norm();
            if (distance < min_distance ){
                best = pl;
                min_distance = distance;
            }
        }
        if (min_distance < max_distance) {
            centroid_right.push_back(pr);
            centroid_left.push_back(best);
        }
    }

    // let's find closest raw mesurments to found centroids

    centroid_left_raw.clear();
    centroid_right_raw.clear();
    // left
    for (const auto & pl : centroid_left)
    {
        double min_distance = std::numeric_limits<double>::max();
        pcl::PointXYZINormal best;
        ASSERT(left.size() == left_raw.size());
        for (int i =0; i < left.size(); i++)
        {
            float distance = (pl.getVector3fMap() - left[i].getVector3fMap()).norm();
            if (min_distance > distance ){
                min_distance = distance;
                best = left_raw[i];
            }
        }
        centroid_left_raw.push_back(best);
    }

    //right
    for (const auto & pl : centroid_right)
    {
        double min_distance = std::numeric_limits<double>::max();
        pcl::PointXYZINormal best;
        ASSERT(right.size() == right_raw.size());
        for (int i =0; i < right.size(); i++)
        {
            float distance = (pl.getVector3fMap() - right[i].getVector3fMap()).norm();
            if (min_distance > distance ){
                min_distance = distance;
                best = right_raw[i];
            }
        }
        centroid_right_raw.push_back(best);
    }

}


void updateDrawingBuffer(const pcl::PointCloud<pcl::PointXYZINormal> &pointcloud,
                         const Eigen::Affine3f &laser_local_calib,
                         std::vector<float>& veritces, std::vector<unsigned int>& indices,
                         int minimum_intensity = 200) {
    veritces.clear();
    indices.clear();
    veritces.reserve(pointcloud.size() * 6);
    indices.reserve(pointcloud.size());

    Eigen::Matrix<float, 4, 4> calib;
    calib <<(0),  (0), (1), (0),
            (0),  (1), (0), (0),
            (-1), (0), (0), (0),
            (0),  (0), (0), (1);
     auto mm = Sophus::SE3f::fitToSE3(laser_local_calib * calib);

     std::cout <<"Calibration for other tools : " << mm.log().transpose() << std::endl;

    //std::cout << calib.matrix()<<std::endl;
    for (int i = 0; i < pointcloud.size(); i++) {
        Eigen::Affine3f encoder_rot(Eigen::Affine3f::Identity());
        const auto &p = pointcloud[i];
        const float angle = -p.normal_x;
        //std::cout << p.normal_x << std::endl;
        Eigen::Affine3f transform(bigUnitCalib::getSE3OfLaser<float>(laser_local_calib.matrix(),angle));
        Eigen::Vector3f pt = transform * p.getVector3fMap();
        veritces.push_back(pt.x());
        veritces.push_back(pt.y());
        veritces.push_back(pt.z());
        if (p.intensity>minimum_intensity)
        {
            //if (pt.x()>1.2) {
                if (p.y > 0) {
                    veritces.push_back(1);
                    veritces.push_back(1);
                    veritces.push_back(1);
                } else {
                    veritces.push_back(1);
                    veritces.push_back(1);
                    veritces.push_back(0);
                }
            //}
        }
        else {
            if (p.y > 0) {
                veritces.push_back(1);
                veritces.push_back(0);
                veritces.push_back(0);
            } else {
                veritces.push_back(0);
                veritces.push_back(1);
                veritces.push_back(0);
            }
        }
        indices.push_back(indices.size());
    }
}

int main(int argc, char **argv) {

    Sophus::SE3d local_calibration_SE3;
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message");
    desc.add_options()("pointcloud,p",po::value<std::string>(),"pointcloud measure");
    desc.add_options()("calibration,c",po::value<std::string>()->default_value("calib.txt"),"calibration");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return false;
    }
    std::cout << "loading pc " << vm["pointcloud"].as<std::string>() << std::endl;
    pcl::PointCloud<pcl::PointXYZINormal> pointcloud;
    pcl::io::loadPCDFile(vm["pointcloud"].as<std::string>(),pointcloud);

    bool gl_dirty = false;

    GLFWwindow *window;
    const char *glsl_version = "#version 130";
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(960, 540, "Simulation", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, cursor_calback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glfwSwapInterval(1);
    if (glewInit() != GLEW_OK) { return -1; }

    GLCall(glClearColor(0.1, 0.1, 0.1, 1));

    Renderer renderer;
    Shader shader(shader_simple_v, shader_simple_f);

    VertexBufferLayout layout;
    layout.Push<float>(3);
    layout.Push<float>(3);

    VertexArray va_co;
    VertexBuffer vb_co(gl_primitives::coordinate_system_vertex.data(),
                       gl_primitives::coordinate_system_vertex.size() * sizeof(float));
    va_co.AddBuffer(vb_co, layout);
    IndexBuffer ib_co(gl_primitives::coordinate_system_indices.data(), gl_primitives::coordinate_system_indices.size());

    std::vector<float> pointcloud_vertices;
    std::vector<unsigned int> pointcloud_indices;
    VertexArray va_pointcloud;
    VertexBuffer vb_pointcloud(pointcloud_vertices.data(),
                           pointcloud_vertices.size() * sizeof(float));
    va_pointcloud.AddBuffer(vb_pointcloud, layout);
    IndexBuffer ib_pointcloud(pointcloud_indices.data(), pointcloud_indices.size());

    std::vector<float> ties_vertices;
    std::vector<unsigned int> ties_indices;
    VertexArray va_ties;
    VertexBuffer vb_ties(ties_vertices.data(),
                         ties_vertices.size() * sizeof(float));
    va_ties.AddBuffer(vb_ties, layout);
    IndexBuffer ib_ties(ties_indices.data(), ties_indices.size());


    std::vector<float> lines_vertices;
    std::vector<unsigned int> lines_indices;
    VertexArray va_lines;
    VertexBuffer vb_lines(lines_vertices.data(),
                          lines_vertices.size() * sizeof(float));
    va_lines.AddBuffer(vb_lines, layout);
    IndexBuffer ib_lines(lines_indices.data(), lines_indices.size());


    Eigen::Affine3f calib(Eigen::Affine3f::Identity());
//    calib.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));

    auto mat = Eigen::Affine3d::Identity();
    mat.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));

    std::vector<float> nns_vertices;
    std::vector<unsigned int> nns_indices;
    VertexArray va_nns;
    VertexBuffer vb_nns(nns_vertices.data(),
                        nns_vertices.size() * sizeof(float));
    va_nns.AddBuffer(vb_nns, layout);
    IndexBuffer ib_nns(nns_indices.data(), nns_indices.size());

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    glm::mat4 glm_test {glm::mat4(1.0f)};

    imgui_vars_old.minimum_intesity = 0;
    while (!glfwWindowShouldClose(window)) {
        if (!(imgui_vars_curr == imgui_vars_old)){
            Eigen::Affine3f calib(Eigen::Affine3f::Identity());
            Eigen::Map<Sophus::Vector6f> m(imgui_vars_curr.laser_config);
            calib = Eigen::Affine3f(Sophus::SE3f::exp(m).matrix());
            updateDrawingBuffer(pointcloud, calib, pointcloud_vertices,pointcloud_indices, imgui_vars_curr.minimum_intesity );

            vb_pointcloud.update(pointcloud_vertices.data(),
                                 pointcloud_vertices.size() * sizeof(float));

            ib_pointcloud.update(pointcloud_indices.data(), pointcloud_indices.size());

            pcl::PointCloud<pcl::PointXYZ> centroid_left;
            pcl::PointCloud<pcl::PointXYZ> centroid_right;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>> centroid_left_raw;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>> centroid_right_raw;

            pcl::PointCloud<pcl::PointXYZ> centroid_left_raw_t;
            pcl::PointCloud<pcl::PointXYZ> centroid_right_raw_t;

            getCentroids(pointcloud, calib, centroid_left, centroid_right,centroid_left_raw,centroid_right_raw,
                         imgui_vars_curr.minimum_intesity, imgui_vars_curr.max_distance);
//            std::cout << "centroid_left " << centroid_left.size() << std::endl;
//            std::cout << "centroid_right " << centroid_right.size() << std::endl;

            for (int i =0; i< centroid_left_raw.size(); i++){
                const auto& pp = centroid_left_raw[i];
                for (const auto p : pp) {
                    const float angle = -p.normal_x;
                    //std::cout << p.normal_x << std::endl;
                    Eigen::Affine3f transform(bigUnitCalib::getSE3OfLaser<float>(calib.matrix(), angle));
                    pcl::PointXYZ pt;
                    pt.getArray3fMap() = transform * p.getVector3fMap();
                    centroid_left_raw_t.push_back(pt);
                }
            }
            for (int i =0; i< centroid_right_raw.size(); i++){
                const auto& pp = centroid_right_raw[i];
                for (const auto p : pp) {
                    const float angle = -p.normal_x;
                    //std::cout << p.normal_x << std::endl;
                    Eigen::Affine3f transform(bigUnitCalib::getSE3OfLaser<float>(calib.matrix(), angle));
                    pcl::PointXYZ pt;
                    pt.getArray3fMap() = transform * p.getVector3fMap();
                    centroid_right_raw_t.push_back(pt);
                }
            }

            ties_vertices.clear();
            ties_indices.clear();
            lines_vertices.clear();
            lines_indices.clear();

            for (const auto &p : centroid_left_raw_t)
            {
                ties_vertices.push_back(p.x);
                ties_vertices.push_back(p.y);
                ties_vertices.push_back(p.z);
                ties_vertices.push_back(1);
                ties_vertices.push_back(0);
                ties_vertices.push_back(0);
                ties_indices.push_back(ties_indices.size());
                //
            }
            for (const auto &p : centroid_right_raw_t)
            {
                ties_vertices.push_back(p.x);
                ties_vertices.push_back(p.y);
                ties_vertices.push_back(p.z);
                ties_vertices.push_back(0);
                ties_vertices.push_back(0);
                ties_vertices.push_back(1);
                ties_indices.push_back(ties_indices.size());
            }
            for (int i =0; i < centroid_left.size(); i++)
            {
                const auto &pr = centroid_right[i];
                const auto &pl = centroid_left[i];

                lines_vertices.push_back(pr.x);
                lines_vertices.push_back(pr.y);
                lines_vertices.push_back(pr.z);
                lines_vertices.push_back(1);
                lines_vertices.push_back(1);
                lines_vertices.push_back(1);
                lines_indices.push_back(lines_indices.size());
                lines_vertices.push_back(pl.x);
                lines_vertices.push_back(pl.y);
                lines_vertices.push_back(pl.z);
                lines_vertices.push_back(1);
                lines_vertices.push_back(1);
                lines_vertices.push_back(1);
                lines_indices.push_back(lines_indices.size());
            }

            vb_ties.update(ties_vertices.data(),
                           ties_vertices.size() * sizeof(float));

            ib_ties.update(ties_indices.data(), ties_indices.size());

            vb_lines.update(lines_vertices.data(), lines_vertices.size()*sizeof(float));
            ib_lines.update(lines_indices.data(), lines_indices.size());
        }
        imgui_vars_old = imgui_vars_curr;

        GLCall(glEnable(GL_DEPTH_TEST));
        GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuiIO &io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        ImGuizmo::Enable(true);
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glm::mat4 proj = glm::perspective(30.f, 1.0f * width / height, 0.05f, 100.0f);
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_0 = glm::rotate(model_translate, float(0 * M_PI), glm::vec3(0.0f, 0.0f, 1.0f));

        glm::mat4 model_rotation_1 = glm::rotate(model_rotation_0, rot_x, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(0.0f, 0.0f, 1.0f));

        shader.Bind(); // bind shader to apply uniform
        // draw reference frame
        GLCall(glPointSize(1));
        GLCall(glLineWidth(1));

        shader.setUniformMat4f("u_MVP", proj * model_rotation_2);

        GLCall(glLineWidth(3));
        renderer.Draw(va_co, ib_co, shader, GL_LINES);
        renderer.Draw(va_pointcloud, ib_pointcloud, shader, GL_POINTS);
        GLCall(glPointSize(10));
        renderer.Draw(va_ties, ib_ties, shader, GL_POINTS);

        GLCall(glLineWidth(1));
        renderer.Draw(va_nns, ib_nns, shader, GL_LINES);

        GLCall(glLineWidth(1));
        renderer.Draw(va_lines, ib_lines, shader, GL_LINES);

        ImGui::Begin("Calibration Demo");
        ImGui::SliderFloat("min_intensity", &imgui_vars_curr.minimum_intesity,100,255);
        ImGui::SliderFloat("max_distance", &imgui_vars_curr.max_distance,0.0f,0.5f);

        ImGui::SliderFloat("laser_param0", &imgui_vars_curr.laser_config[0],-0.1f,0.3f);
        ImGui::SliderFloat("laser_param1", &imgui_vars_curr.laser_config[1],-0.1f,0.1f);
        ImGui::SliderFloat("laser_param2", &imgui_vars_curr.laser_config[2],-0.1f,0.1f);
        ImGui::SliderFloat("laser_param3", &imgui_vars_curr.laser_config[3],-0.1f,0.1f);
        ImGui::SliderFloat("laser_param4", &imgui_vars_curr.laser_config[4],-0.1f,0.1f);
        ImGui::SliderFloat("laser_param5", &imgui_vars_curr.laser_config[5],-0.1f,0.1f);

        if(ImGui::Button("test_cost")){
            Eigen::Affine3f calib(Eigen::Affine3f::Identity());
            Eigen::Map<Sophus::Vector6f> m(imgui_vars_curr.laser_config);
            calib = Eigen::Affine3f(Sophus::SE3f::exp(m).matrix());

            pcl::PointCloud<pcl::PointXYZ> centroid_left;
            pcl::PointCloud<pcl::PointXYZ> centroid_right;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>> centroid_left_raw;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>> centroid_right_raw;
            pcl::PointCloud<pcl::PointXYZINormal> centroid_left_raw2;
            pcl::PointCloud<pcl::PointXYZINormal> centroid_right_raw2;

            getCentroids(pointcloud, calib, centroid_left, centroid_right,centroid_left_raw,centroid_right_raw,
                         imgui_vars_curr.minimum_intesity,imgui_vars_curr.max_distance);

            getCentroids(pointcloud, calib, centroid_left, centroid_right,centroid_left_raw2,centroid_right_raw2,
                         imgui_vars_curr.minimum_intesity,imgui_vars_curr.max_distance);

            Eigen::Map<Sophus::Vector6f> params (imgui_vars_curr.laser_config);
            local_calibration_SE3 = Sophus::SE3d::exp(params.cast<double>());


            ASSERT(centroid_left_raw.size()==centroid_right_raw.size());
            Eigen::Vector3d cost;
            for (int i =0; i < centroid_left_raw.size(); i++){
                double residiuals[3];
                auto c = bigUnitCalib::OptimizeLaser3DBlobs(centroid_left_raw[i], centroid_right_raw[i]);
                c(local_calibration_SE3.data(),residiuals);
                Eigen::Map<Eigen::Vector3d> cost_one(residiuals);
                std::cout << centroid_left_raw[i].size() <<": "<< centroid_right_raw[i].size() << std::endl;
                std::cout << cost_one.transpose() << std::endl;
                cost = cost + cost_one;
            }
            std::cout << "cost " << cost << std::endl;
//            for (int i =0; i < centroid_left_raw.size(); i++){
//                ceres::LossFunction *loss = nullptr;//new ceres::CauchyLoss(0.05);
//                ceres::CostFunction *cost_function =
//                        bigUnitCalib::OptimizeLaserLocalPoseWithFix::Create(
//                                centroid_left_raw2.points[i].getArray3fMap().cast<double>(),
//                                centroid_right.points[i].getArray3fMap().cast<double>(),
//                                centroid_left_raw2.points[i].normal_x);
//                problem.AddResidualBlock(cost_function, loss, local_calibration_SE3.data());
//            }
////
//            for (int i =0; i < centroid_right_raw.size(); i++){
//                ceres::LossFunction *loss = nullptr;//new ceres::CauchyLoss(0.05);
//                ceres::CostFunction *cost_function =
//                        bigUnitCalib::OptimizeLaserLocalPoseWithFix::Create(
//                                centroid_right_raw2.points[i].getArray3fMap().cast<double>(),
//                                centroid_left.points[i].getArray3fMap().cast<double>(),
//                                centroid_right_raw2.points[i].normal_x);
//                problem.AddResidualBlock(cost_function, loss, local_calibration_SE3.data());
//            }




        }


        if(ImGui::Button("optimize laser")){
            Eigen::Affine3f calib(Eigen::Affine3f::Identity());
            Eigen::Map<Sophus::Vector6f> m(imgui_vars_curr.laser_config);
            calib = Eigen::Affine3f(Sophus::SE3f::exp(m).matrix());

            pcl::PointCloud<pcl::PointXYZ> centroid_left;
            pcl::PointCloud<pcl::PointXYZ> centroid_right;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>> centroid_left_raw;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>> centroid_right_raw;
            pcl::PointCloud<pcl::PointXYZINormal> centroid_left_raw2;
            pcl::PointCloud<pcl::PointXYZINormal> centroid_right_raw2;

            getCentroids(pointcloud, calib, centroid_left, centroid_right,centroid_left_raw,centroid_right_raw,
                         imgui_vars_curr.minimum_intesity,imgui_vars_curr.max_distance);

            getCentroids(pointcloud, calib, centroid_left, centroid_right,centroid_left_raw2,centroid_right_raw2,
                         imgui_vars_curr.minimum_intesity,imgui_vars_curr.max_distance);


            Eigen::Map<Sophus::Vector6f> params (imgui_vars_curr.laser_config);
            local_calibration_SE3 = Sophus::SE3d::exp(params.cast<double>());
            ceres::Problem problem;
            problem.AddParameterBlock(local_calibration_SE3.data(), Sophus::SE3d::num_parameters, new LocalParameterizationSE3());

            ASSERT(centroid_left_raw.size()==centroid_right_raw.size());
            for (int i =0; i < centroid_left_raw.size(); i++){
                ceres::LossFunction *loss =nullptr;
                ceres::CostFunction *cost_function =
                        bigUnitCalib::OptimizeLaser3DBlobs::Create(
                                centroid_left_raw[i],
                                centroid_right_raw[i]);
                problem.AddResidualBlock(cost_function, loss, local_calibration_SE3.data());
            }
            for (int i =0; i < centroid_left_raw.size(); i++){
                ceres::LossFunction *loss = nullptr;// new ceres::CauchyLoss(0.01);
                ceres::CostFunction *cost_function =
                        bigUnitCalib::OptimizeLaserLocalPoseWithFix::Create(
                                centroid_left_raw2.points[i].getArray3fMap().cast<double>(),
                                centroid_left.points[i].getArray3fMap().cast<double>(),
                                centroid_left_raw2.points[i].normal_x);
                problem.AddResidualBlock(cost_function, loss, local_calibration_SE3.data());
                if (i>0) break;
            }
////
//            for (int i =0; i < centroid_right_raw.size(); i++){
//                ceres::LossFunction *loss = nullptr;//new ceres::CauchyLoss(0.05);
//                ceres::CostFunction *cost_function =
//                        bigUnitCalib::OptimizeLaserLocalPoseWithFix::Create(
//                                centroid_right_raw2.points[i].getArray3fMap().cast<double>(),
//                                centroid_left.points[i].getArray3fMap().cast<double>(),
//                                centroid_right_raw2.points[i].normal_x);
//                problem.AddResidualBlock(cost_function, loss, local_calibration_SE3.data());
//            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 500;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
            std::cout << "deltas "<< (params - local_calibration_SE3.log().cast<float>()).transpose() << std::endl;


            params = local_calibration_SE3.log().cast<float>();

        }
        if(ImGui::Button("export_calib")){
            Eigen::Map<Sophus::Vector6f> params (imgui_vars_curr.laser_config);
            const auto c = Sophus::SE3f::exp(params);
            m3d_utils::saveVec6(vm["calibration"].as<std::string>(), c.log().cast<double>());
        }

        if(ImGui::Button("import_calib")){
            const auto m_raw = m3d_utils::loadVec6(vm["calibration"].as<std::string>()).cast<float>();

            std::cout << "imported calibration: \n" << m_raw << std::endl;
//            const auto c = Sophus::SE3f(m_raw);
            Eigen::Map<Sophus::Vector6f> params (imgui_vars_curr.laser_config);
            params = m_raw;
        }

        if(ImGui::Button("reset_calib")){
            Eigen::Map<Sophus::Vector6f> params (imgui_vars_curr.laser_config);
            params = Sophus::Vector6f::Zero();
        }

        if(ImGui::Button("export")){
            Eigen::Affine3f calib(Eigen::Affine3f::Identity());
            Eigen::Map<Sophus::Vector6f> m(imgui_vars_curr.laser_config);
            calib = Eigen::Affine3f(Sophus::SE3f::exp(m).matrix());
            pcl::PointCloud<pcl::PointXYZI> ouput;
            ouput.reserve(pointcloud.size());
            for (int i = 0; i < pointcloud.size(); i++) {
                Eigen::Affine3f encoder_rot(Eigen::Affine3f::Identity());
                const auto &p = pointcloud[i];
                const float angle = -p.normal_x;
                //std::cout << p.normal_x << std::endl;
                Eigen::Affine3f transform(bigUnitCalib::getSE3OfLaser<float>(calib.matrix(),angle));
                pcl::PointXYZI pt;
                pt.getArray3fMap() = transform * p.getVector3fMap();
                ouput.push_back(pt);
            }
            pcl::io::savePCDFile("pointcloud.pcd", ouput);

        }
        ImGui::End();
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
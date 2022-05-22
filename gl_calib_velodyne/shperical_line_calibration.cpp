//
// Created by michal on 24.06.2021.
//

//pcl


// optimizing and multihtrading
#include "tbb/tbb.h"
#include <mutex>

// opengl
#include "GL/glwrapper.h"
#include "ImGuizmo/ImGuizmo.h"


#include "cost_fun.h"
#include "line_detector.h"

template<typename T> Sophus::SE3<T>  getSEFromParams(const T* const params)
{
    Eigen::Map<const Eigen::Matrix<T,6,1>> eigen_laser_params(params);
    Sophus::SE3<T> TT = Sophus::SE3<T>::exp(eigen_laser_params);
    return TT;
}
glm::vec2 clicked_point;
glm::vec3 view_translation{ 0,0,-30 };
float rot_x =0.0f;
float rot_y =0.0f;
bool drawing_buffer_dirty = true;
void cursor_calback(GLFWwindow* window, double xpos, double ypos)
{
    ImGuiIO& io = ImGui::GetIO();
    if(!io.WantCaptureMouse) {
        const glm::vec2 p{-xpos, ypos};
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

cv::Vec2f getShpericalFromLatLon(float alpha2, float omega2, int cols, int rows)
{
    double xx2 = 1.0*cols * alpha2 /(2.0*M_PI) + cols/2;
    double yy2 = 1.0*(rows * (-omega2) /(M_PI)) + rows/2;
//    xx2 = std::min(0., xx2);
//    xx2 = std::max(double(rows), xx2);
//    yy2 = std::min(0., yy2);
//    yy2 = std::max(double(rows), yy2);
    return cv::Vec2f(xx2,yy2);
}

float deg2rad(float d)
{
    return M_PI* d / 180.0;
}
template<typename T>
Eigen::Matrix<T,6,6> getPluckerLineTransformer(const Eigen::Matrix<T,4,4>& rt)
{
    Eigen::Matrix<T,6,6> matrix;
    matrix.setZero();
    const Eigen::Matrix<T,3,3> R = rt.template block<3, 3>(0, 0);
    const T a = rt(0,3);
    const T b = rt(1,3);
    const T c = rt(2,3);
    Eigen::Matrix<T,3,3> skew_t;
    skew_t << T(0), -c, b,
            c,T(0),-a,
            -b,a,T(0);
    matrix.template block<3,3>(0,0) = R;
    matrix.template block<3,3>(3,0).setZero();
    matrix.template block<3,3>(0,3) = skew_t*R;
    matrix.template block<3,3>(3,3) = R;
    return matrix;
}

using Vector6d=Eigen::Matrix<double,6,1>;

void DrawPlucker( const Vector6d& plucker, std::vector<float>&vertex,std::vector<unsigned int>&indices)
{
    vertex.clear();
    indices.clear();

    vertex.insert(std::end(vertex),{0,0,0});
    vertex.insert(std::end(vertex),{0,0,0});

    vertex.insert(std::end(vertex),{10.f * (float)plucker[0],10.f * (float)plucker[1],10.f * (float)plucker[2]});
    vertex.insert(std::end(vertex),{1,0,0});

    vertex.insert(std::end(vertex),{10.f * (float)plucker[3],10.f * (float)plucker[4],10.f * (float)plucker[5]});
    vertex.insert(std::end(vertex),{0,1,0});

    indices.push_back(0);
    indices.push_back(2);
    indices.push_back(0);
    indices.push_back(1);

}
Vector6d getPlucker(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2){
    Eigen::Vector3d plucker_l = x1 - x2;
    plucker_l = plucker_l/ plucker_l.norm();
    Eigen::Vector3d plucker_m = x2.cross(plucker_l);
//    if (plucker_m.z() < 0) {
//        plucker_m = -plucker_m;
//        plucker_l = -plucker_l;
//    }
    Vector6d plucker;
    plucker.block<3,1>(0,0) = plucker_m;
    plucker.block<3,1>(3,0) = plucker_l;
    return plucker;
}


struct CostFunctor{
    const Eigen::Vector2d line_in_spherical_image;
    const Vector6d line_in_scene; //
    Eigen::Vector3d moment_observed;

    CostFunctor(const Eigen::Vector2d& line_in_spherical_image, const Vector6d& line_in_scene) :
            line_in_spherical_image(line_in_spherical_image), line_in_scene(line_in_scene){

        const Eigen::Matrix3d transfer =(Eigen::AngleAxisd(-line_in_spherical_image.x()+M_PI/2, Eigen::Vector3d::UnitZ())*
                                         Eigen::AngleAxisd(line_in_spherical_image.y(), Eigen::Vector3d::UnitY())).matrix() ;

        moment_observed = transfer.col(2);

    }

    template <typename T>
    bool operator()(const T* const posetangent,
                    T* residuals) const {
        Sophus::SE3<T> params1 = getSEFromParams<T>(posetangent);
        Eigen::Matrix<T,4,4> params1_inv = params1.inverse().matrix();
        Eigen::Matrix<T,6,1> plucker_local = getPluckerLineTransformer<T>(params1_inv) * line_in_scene;
        T z_dist = plucker_local.template block<3, 1>(0, 0).norm();

        Eigen::Matrix<T,3,1> plucker_moment = plucker_local.template block<3, 1>(0, 0);
        Eigen::Matrix<T,3,1> plucker_moment_norm = plucker_local.template block<3, 1>(0, 0)/z_dist;


        Eigen::Matrix<T,3,1> plucker_dir_norm = plucker_local.template block<3, 1>(3, 0);

        auto moment_observed_T = moment_observed.cast<T>();
        Eigen::Matrix<T,3,1> l_moment = moment_observed_T.cross(plucker_moment);
        //Eigen::Matrix<T,3,1> l_dir = moment_observed_T.cross(plucker_dir_norm);

        residuals[0] = l_moment.squaredNorm();
//        residuals[1] = params1.translation().z() - 0.2;
//        residuals[2] = 0.01*params1.translation().x();
//        residuals[3] = 0.01*params1.translation().y();

//        residuals[2] = 0.001*params1.translation().y();
        //residuals[3] = 0.2-params1.translation().z();
        //residuals[2] = ceres::abs(plucker_moment_norm.y() - moment_observed_T.y());
        //residuals[3] = ceres::abs(plucker_moment_norm.z() - moment_observed_T.z());


        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& line_in_spherical_image, const Vector6d& line_in_scene   ) {
        return (new ceres::AutoDiffCostFunction<CostFunctor, 1, 6>(
                new CostFunctor(line_in_spherical_image, line_in_scene)));
//        return (new ceres::NumericDiffCostFunction<CostFunctor,ceres::CENTRAL, 2, 6>(
//                new CostFunctor(line_in_spherical_image, line_in_scene)));

    }
};

void addLineToBuffer(std::vector<float>& draw_data_buffer_test, std::vector<unsigned int>& draw_data_test_indices,
                     const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3f& color){
    //0
    draw_data_buffer_test.insert(std::end(draw_data_buffer_test),{(float)x1.x(),(float)x1.y(),(float)x1.z()});
    draw_data_buffer_test.insert(std::end(draw_data_buffer_test),{color.x(),color.y(),color.z()});
    draw_data_test_indices.push_back(draw_data_test_indices.size());
    //1
    draw_data_buffer_test.insert(std::end(draw_data_buffer_test),{(float)x2.x(),(float)x2.y(),(float)x2.z()});
    draw_data_buffer_test.insert(std::end(draw_data_buffer_test),{color.x(),color.y(),color.z()});
    draw_data_test_indices.push_back(draw_data_test_indices.size());

}
cv::Mat& drawGreatCircle(cv::Mat &sphere_img, double azimuth, double inclination, cv::Vec3b color)
{
    const float increment = 0.001*M_PI;
    for (float longitude0 =-M_PI; longitude0 < M_PI; longitude0+=increment)
    {
        const float longitude1 = longitude0 + increment;
        const float lattidue_rad0 = std::atan(std::tan(inclination) * std::sin(longitude0 - azimuth));
        const float lattidue_rad1 = std::atan(std::tan(inclination) * std::sin(longitude1 - azimuth));
        const cv::Vec2f xy0 = getShpericalFromLatLon(longitude0, lattidue_rad0, sphere_img.cols, sphere_img.rows);
        const cv::Vec2f xy1 = getShpericalFromLatLon(longitude1, lattidue_rad1, sphere_img.cols, sphere_img.rows);
        cv::line(sphere_img, cv::Vec2i(xy0),cv::Vec2i(xy1), color,2);
    }
    return sphere_img;
}

void writeLines(const std::vector<Eigen::Vector2f>& detected_lines,
                const std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>>& lines,
                const Eigen::Matrix4f & camera, const std::string& workspace)
{
    boost::property_tree::ptree pt;
    pt.put("size", detected_lines.size());

    pt.put("camera.00", camera(0,0));
    pt.put("camera.01", camera(0,1));
    pt.put("camera.02", camera(0,2));
    pt.put("camera.03", camera(0,3));

    pt.put("camera.10", camera(1,0));
    pt.put("camera.11", camera(1,1));
    pt.put("camera.12", camera(1,2));
    pt.put("camera.13", camera(1,3));

    pt.put("camera.20", camera(2,0));
    pt.put("camera.21", camera(2,1));
    pt.put("camera.22", camera(2,2));
    pt.put("camera.23", camera(2,3));

    pt.put("camera.30", camera(3,0));
    pt.put("camera.31", camera(3,1));
    pt.put("camera.32", camera(3,2));
    pt.put("camera.33", camera(3,3));

    for (int i =0; i < detected_lines.size(); i++)
    {
        pt.put("2d.lines."+std::to_string(i)+".0", detected_lines[i][0]);
        pt.put("2d.lines."+std::to_string(i)+".1", detected_lines[i][1]);
        if (i < lines.size()) {
            std::ostringstream oss1;
            oss1 << lines[i].first.x() << " " << lines[i].first.y() << " " << lines[i].first.z();
            pt.put("3d.lines." + std::to_string(i) + ".first", oss1.str());
            std::ostringstream oss2;
            oss2<< lines[i].second.x() <<" " << lines[i].second.y() << " "<< lines[i].second.z();
            pt.put("3d.lines."+std::to_string(i)+".second", oss2.str());
        }

    }
    boost::property_tree::write_json(workspace+"/detected_lines.json", pt);
}
void loadLines(std::vector<Eigen::Vector2f>& detected_lines, std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>>& lines,
               Eigen::Matrix4f & camera, const std::string& workspace){
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(workspace+"/detected_lines.json", pt);
    detected_lines.clear();
    lines.clear();
    detected_lines.resize(pt.get<int>("size"));
    lines.resize(pt.get<int>("size"));

    camera(0,0) =pt.get<float>("camera.00");
    camera(0,1) =pt.get<float>("camera.01");
    camera(0,2) =pt.get<float>("camera.02");
    camera(0,3) =pt.get<float>("camera.03");

    camera(1,0) =pt.get<float>("camera.10");
    camera(1,1) =pt.get<float>("camera.11");
    camera(1,2) =pt.get<float>("camera.12");
    camera(1,3) =pt.get<float>("camera.13");

    camera(2,0) =pt.get<float>("camera.20");
    camera(2,1) =pt.get<float>("camera.21");
    camera(2,2) =pt.get<float>("camera.22");
    camera(2,3) =pt.get<float>("camera.23");

    camera(3,0) =pt.get<float>("camera.30");
    camera(3,1) =pt.get<float>("camera.31");
    camera(3,2) =pt.get<float>("camera.32");
    camera(3,3) =pt.get<float>("camera.33");

    for (int i =0; i < detected_lines.size(); i++)
    {
        auto opt = pt.get_child_optional("2d.lines."+std::to_string(i));
        if (opt) {
            detected_lines[i][0] = pt.get<float>("2d.lines." + std::to_string(i) + ".0");
            detected_lines[i][1] = pt.get<float>("2d.lines." + std::to_string(i) + ".1");
        }
        std::string s1= pt.get<std::string>("3d.lines."+std::to_string(i)+".first");
        std::string s2 = pt.get<std::string>("3d.lines."+std::to_string(i)+".second");

        std::stringstream oss1(s1);
        std::stringstream oss2(s2);

        oss1>> lines[i].first.x();
        oss1>> lines[i].first.y();
        oss1>> lines[i].first.z();

        oss2>> lines[i].second.x();
        oss2>> lines[i].second.y();
        oss2>> lines[i].second.z();

    }
}
bool updateDrawingBufferWithColor(std::vector<float> &draw_buffer_cloud, std::vector<unsigned int> &draw_buffer_cloud_indices,
                                  const pcl::PointCloud<pcl::PointXYZRGB> &pc,
                                  const cv::Mat &sphere,  const Eigen::Matrix4d& sphere_odometry){
    draw_buffer_cloud.clear();
    draw_buffer_cloud_indices.clear();

    std::cout << "sphere_odometry " << std::endl;
    std::cout << sphere_odometry << std::endl;
    for (const auto p: pc)
    {

        draw_buffer_cloud.insert(draw_buffer_cloud.end(), { p.x,p.y,p.z });
        if (!sphere.empty())
        {
            Eigen::Matrix<double,1,4> pt{p.x,p.y,p.z,1.0};
            Eigen::Matrix<double,4,4> rot  =  Eigen::Matrix<double,4,4>::Identity();
            rot.block<3,3>(0,0) = (Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ())).matrix();
            Eigen::Matrix<double,1,4> pt_cam = rot*sphere_odometry*pt.transpose();

            double alpha2 = std::atan2(pt_cam.x(),pt_cam.y());
            double r = std::sqrt(pt_cam.x()*pt_cam.x() + pt_cam.y()*pt_cam.y() + pt_cam.z()*pt_cam.z());
            double omega2 = std::asin(pt_cam.z()/r);

            double xx2 = 1.0*sphere.cols * alpha2 /(2.0*M_PI) + sphere.cols/2;
            double yy2 = 1.0*(sphere.rows * (-omega2) /(M_PI)) + sphere.rows/2;

            if (xx2>0 && yy2>0 && xx2 <sphere.cols && yy2 < sphere.rows)
            {
                const auto pix = sphere.at<cv::Vec3b>(yy2,xx2);
                draw_buffer_cloud.insert(draw_buffer_cloud.end(), {(1.0f*pix[2])/256.0f, (1.0f*pix[1])/256.0f, (1.0f*pix[0])/256.0f});
            }
        }
        else {
            draw_buffer_cloud.insert(draw_buffer_cloud.end(), {1.0f * p.r / 255, 1.0f * p.g / 255, 1.0f * p.g / 255});
        }
        draw_buffer_cloud_indices.push_back(draw_buffer_cloud_indices.size());
    }
}
int main(int argc, char **argv) {

    std::string workspace(argv[1]);
    std::cout << "workspace " << workspace << std::endl;
    std::vector<std::string> extra_data{
            workspace
//        "/mnt/540C28560C283580/dane_plecak/tests/palak/test_side/",
//        "/mnt/540C28560C283580/dane_plecak/tests/palak/test_in_0.25",
//        "/mnt/540C28560C283580/dane_plecak/tests/palak/test_out_1"
    };
//    std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> lines{
//            {{5,4,0},{5,-4,0}},
//            {{5,4,3},{5,-4,3}},
//            {{5,4,0},{5,4,3}},
//            {{5,-4,0},{5,-4,3}},
//    };

    std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> lines_3d{
//            {{ 20.123195648193,0.494958996773,0.663995981216 },{20.132989883423,-4.984799861908,0.668256998062}},
//            {{26.085592269897,0.671194970608,0.645429015160},{26.081626892090,-4.958352088928,0.642925977707}},
//            {{19.998794555664,6.550938129425,-1.342049002647},{9.006967544556,6.566266059875,-1.356233000755}},
//            {{14.487122535706,-4.847480773926,0.645928025246},{20.156965255737,-4.917168140411,0.578855991364}},
//            {{20.137187957764,0.498082011938,1.042235016823},{14.730258941650,0.390141993761,1.088276982307}}
    };
    int gizmed_line_id = -1;

    std::vector<Eigen::Vector3f> colors{{1,0,0},{0,1,0},{0,0,1},{1,0,1},{1,1,0},{1,1,1},{0,0,0}};

    std::vector<Eigen::Vector2f> detected_lines;
    std::vector<std::vector<Eigen::Vector2d>> points_in_line;

    GLFWwindow* window;
    const char* glsl_version = "#version 130";
    /* Initialize the library */
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(960, 540, "Backpack calib", NULL, NULL);
    if (!window){glfwTerminate(); return -1;}
    glfwSwapInterval(1);
    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, cursor_calback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (glewInit() != GLEW_OK){glfwTerminate(); return -1;}
    //BACKGOUND!
    GLCall(glClearColor(0.4,0.4,0.4,1));
    //GLCall(glClearColor(1,1,1,1));

    VertexBufferLayout layout;
    layout.Push<float>(3);
    layout.Push<float>(3);

    VertexArray va_co;
    VertexBuffer vb_co(gl_primitives::coordinate_system_vertex.data(), gl_primitives::coordinate_system_vertex.size() * sizeof(float));
    va_co.AddBuffer(vb_co, layout);
    IndexBuffer ib_co(gl_primitives::coordinate_system_indices.data(), gl_primitives::coordinate_system_indices.size());

    std::vector<float> draw_buffer_line;
    std::vector<unsigned int> draw_line_indices;


    VertexArray va_lines;
    VertexBuffer vb_lines(draw_buffer_line.data(), draw_buffer_line.size() * sizeof(float));
    va_lines.AddBuffer(vb_lines, layout);
    IndexBuffer ib_lines(draw_line_indices.data(), draw_line_indices.size());

    std::vector<float> draw_buffer_cloud;
    std::vector<unsigned int> draw_buffer_cloud_indices;
    pcl::PointCloud<pcl::PointXYZINormal> pc_raw;
    pcl::PointCloud<pcl::PointXYZRGB> pc;

    pcl::io::loadPCDFile(workspace+"/1635402399_pointcloud_raw.pcd", pc_raw);

    // apply calibration
    Eigen::Matrix4f bighead_calib = m3d_utils::loadMat("calib.txt").cast<float>();
    pcl::PointCloud<pcl::PointXYZI> ouput;
    pc.reserve(pc_raw.size());
    for (int i = 0; i < pc_raw.size(); i++) {
        Eigen::Affine3f encoder_rot(Eigen::Affine3f::Identity());
        const auto &p = pc_raw[i];
        const float angle = -p.normal_x;
        //std::cout << p.normal_x << std::endl;
        Eigen::Affine3f transform(bigUnitCalib::getSE3OfLaser<float>(bighead_calib,angle));
        pcl::PointXYZRGB pt;
        pt.getArray3fMap() = transform * p.getVector3fMap();
        pt.r = p.intensity*255;
        pt.g = p.intensity*255;
        pt.b = p.intensity*255;
        pc.push_back(pt);
    }

    //pcl::io::loadPCDFile("/tmp/cloud.pcd", pc);

    //pcl::io::loadPCDFile("/mnt/540C28560C283580/dane_plecak/tests/front/25/cloud.pcd", pc);

    updateDrawingBufferWithColor(draw_buffer_cloud, draw_buffer_cloud_indices, pc, cv::Mat(), Eigen::Matrix4d::Identity());
    VertexArray va_pointloud;
    VertexBuffer vb_pointcloud(draw_buffer_cloud.data(), draw_buffer_cloud.size() * sizeof(float));
    va_pointloud.AddBuffer(vb_pointcloud, layout);
    IndexBuffer ib_pointcloud(draw_buffer_cloud_indices.data(), draw_buffer_cloud_indices.size());


    Shader shader(shader_simple_v, shader_simple_f);
    Renderer renderer;

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init(glsl_version);
    glm::mat4 camera_matrix {glm::rotate(glm::mat4(1.0f), deg2rad(0.0f), glm::vec3(1, 0, 0))};
    glm::mat4 utility_matrix {glm::rotate(glm::mat4(1.0f), deg2rad(0.0f), glm::vec3(1, 0, 0))};

    const auto load_init = [&](){Eigen::Map<Eigen::Matrix4f> mat_map(&camera_matrix[0][0]);
        mat_map = Eigen::Matrix4f::Identity();
//    mat_map << 0.8264643549919128, -0.5520190596580505, -0.11059680581092834, 0.0,
//            0.5173956155776978, 0.6672905683517456, 0.535747230052948, 0.0,
//            -0.2219424992799759, -0.4999983310699463, 0.837104082107544, 2.0,
//            0.0, 0.0, 0.0, 1.0;

//        mat_map << 0.84239262342453, -0.42403846979141235, 0.33251482248306274, 0.0,
//                0.44319528341293335, 0.8962000608444214, 0.020085996016860008, 0.0,
//                -0.3065170347690582, 0.13044871389865875, 0.942884087562561, 2.0,
//                0.0, 0.0, 0.0, 1.0;

//        mat_map << -0.059633541852235794, -0.9982203245162964, 0.0, 20.283273696899414,
//                0.9982203245162964, -0.059633541852235794, 0.0, -2.5287961959838867,
//                0.0, 0.0, 1.0, -0.46487948298454285,
//                0.0, 0.0, 0.0, 1.0;
    };
    load_init();
    const int cols = 3840;
    const int rows = 1920;
    cv::Mat sphere_img1 = cv::Mat::zeros(rows, cols, CV_8UC3);

    //cv::Mat sphere_img_color = cv::imread("/home/michal/sphere_test.png");
    //cv::Mat sphere_img_color = cv::imread("/mnt/540C28560C283580/dane_plecak/tests/palak/test_25/cv.png");
    //cv::Mat sphere_img_color = cv::imread("/mnt/540C28560C283580/dane_plecak/tests/front/35/cv.png");
    cv::Mat sphere_img_color = cv::imread(workspace+"/1635402399_ladybugPostProcessing-panoramic-0-radius_50.000000.jpg");


    sphere_img1 = sphere_img_color.clone();

    cv::Mat input_gray;

    cv::cvtColor(sphere_img_color,input_gray, cv::COLOR_BGR2GRAY );


    bool draw_det {true};
    Texture texture_camera1(sphere_img1);
    bool camera_gizmo = false;
    while (!glfwWindowShouldClose(window)) {
        draw_buffer_line.clear();
        draw_line_indices.clear();
        for (int i =0; i < lines_3d.size(); i++) {
            const auto &p = lines_3d[i];
            const auto &c = colors[i];
            addLineToBuffer(draw_buffer_line, draw_line_indices, p.first, p.second, c);
        }
        vb_lines.update(draw_buffer_line.data(), draw_buffer_line.size() * sizeof(float));
        ib_lines.update(draw_line_indices.data(), draw_line_indices.size());

        std::vector<Vector6d> pluckers;
        for (const auto &p : lines_3d)
        {
            pluckers.push_back(getPlucker(p.first,p.second));
        }

        GLCall(glEnable(GL_DEPTH_TEST));
        GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();

        ImGui::NewFrame();

        std::stringstream oss;
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

        ImGui::Begin("OBS");
        ImGui::Checkbox("camera_gizmo",&camera_gizmo);
        if (ImGui::Button("reset"))load_init();
        ImGui::Checkbox("draw_detection", &draw_det);
        for (int i =0; i< detected_lines.size(); i++)
        {
            ImGui::SliderFloat2(("line_"+std::to_string(i)).c_str(), detected_lines[i].data(), -M_PI, M_PI, "%.3f" );
            ImGui::SameLine();
            if(ImGui::Button(("set_g"+std::to_string(i)).c_str())){
                Eigen::Map<Eigen::Matrix4f> utility_matrix_map(&utility_matrix[0][0]);
                Eigen::Matrix4d utility_matrix_or = m3d_utils::orthogonize(utility_matrix_map.cast<double>());
                //utility_matrix_or = utility_matrix_or.inverse();
                lines_3d[i].first = Eigen::Affine3d(utility_matrix_or) * Eigen::Vector3d({-10, 0, 0});
                lines_3d[i].second = Eigen::Affine3d(utility_matrix_or) * Eigen::Vector3d({10, 0, 0});
            }
            ImGui::SameLine();
            if(ImGui::Button(("get_g"+std::to_string(i)).c_str())){
                Eigen::Map<Eigen::Matrix4f> utility_matrix_map(&utility_matrix[0][0]);

                Eigen::Matrix4d mat(Eigen::Matrix4d::Identity());
                Eigen::Vector3d m = lines_3d[i].first - lines_3d[i].second;
                mat(0,3) = lines_3d[i].first.x() - m.x() / 2;
                mat(1,3) = lines_3d[i].first.y() - m.y() / 2;
                mat(2,3) = lines_3d[i].first.z() - m.z() / 2;

                mat.block<3,1>(0,0) = m/m.norm();
                mat.block<3,1>(0,2) = Eigen::Vector3d{0,0,1};
                mat.block<3,1>(0,1) = mat.block<3,1>(0,2).cross(mat.block<3,1>(0,0));
                utility_matrix_map = mat.cast<float>();
                //utility_matrix_or = m3d_utils::orthogonize(utility_matrix_map.cast<double>());
                //utility_matrix_or = utility_matrix_or.inverse();
            }
            ImGui::SameLine();
            if(ImGui::Button(("get_from_asc"+std::to_string(i)).c_str())){
                const std::string fn(workspace + "/line_" + std::to_string(i) + ".asc");
                auto line = spherical_calib_utils::getLineFromASC(fn);
                lines_3d[i] = line;
            }


        }
        if (ImGui::Button("add")) {
            detected_lines.push_back({0,0});
            lines_3d.push_back(std::pair<Eigen::Vector3d,Eigen::Vector3d>({{0, 0, 0}, {1, 0, 0}}));
        };
        ImGui::SameLine();

        if (ImGui::Button("load")) {
            Eigen::Matrix4f m;
            loadLines(detected_lines, lines_3d, m, workspace);
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            gizmo_mat = m;
        }
        ImGui::SameLine();
        if (ImGui::Button("save")) {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            writeLines(detected_lines, lines_3d, gizmo_mat , workspace);
        }
        ImGui::SameLine();
        if (ImGui::Button("print_lines_from3d")) {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            for (int i = 0; i < lines_3d.size(); i++) {
                const auto &p = lines_3d[i];
                const auto &c = colors[i];
                Vector6d plucker_local = getPluckerLineTransformer<double>(gizmo_mat_t.inverse()) * pluckers[i];
                const float z_dist = plucker_local.block<3, 1>(0, 0).norm();
                const float inclination = std::acos(plucker_local.z() / z_dist);
                const float azimuth = std::atan2(plucker_local.x(), plucker_local.y());
                std::cout << "i" << i << " " << azimuth << " " << inclination << std::endl;
            }
        }
        if (ImGui::Button("color"))
        {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            updateDrawingBufferWithColor(draw_buffer_cloud, draw_buffer_cloud_indices, pc, sphere_img_color, gizmo_mat_t.inverse());

            vb_pointcloud.update(draw_buffer_cloud.data(), draw_buffer_cloud.size() * sizeof(float));
            ib_pointcloud.update(draw_buffer_cloud_indices.data(), draw_buffer_cloud_indices.size());
        }
        ImGui::SameLine();
        if (ImGui::Button("uncolor"))
        {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            updateDrawingBufferWithColor(draw_buffer_cloud, draw_buffer_cloud_indices, pc, cv::Mat(), gizmo_mat_t.inverse());

            vb_pointcloud.update(draw_buffer_cloud.data(), draw_buffer_cloud.size() * sizeof(float));
            ib_pointcloud.update(draw_buffer_cloud_indices.data(), draw_buffer_cloud_indices.size());
        }


//        if (true) {
//            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&matrix[0][0]);
//            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
//            Sophus::SE3d gizmo_SE3(gizmo_mat_t);
//            Vector6d gizmo_se3 = gizmo_SE3.log();
//
//            for (int i = 0; i < detected_lines.size(); i++) {
//                const Eigen::Vector2d line_in_image = detected_lines[i].cast<double>();
//                const Vector6d line_in_3d = pluckers[i];
//
//                CostFunctor cost_function = CostFunctor(line_in_image, line_in_3d);
//                Eigen::Vector4d residuals;
//                cost_function(gizmo_se3.data(), residuals.data());
//                oss << i << ":" << residuals.transpose().format(CleanFmt)<<"\n";
//            }
//
//            ImGui::Text(oss.str().c_str());
//        }
        if (ImGui::Button("Detect lines in image"))
        {
            detected_lines.clear();
            lines_3d.clear();
            line_detector::Setup setup;
            std::vector<Eigen::Vector2d> pp = line_detector::findGreatCirclesInImage(sphere_img_color, setup);
            for (int i =0; i<pp.size(); i++ )
            {
                detected_lines.push_back(pp[i].cast<float>());
                lines_3d.push_back(std::pair<Eigen::Vector3d,Eigen::Vector3d>({{0, 0, 1.0 * i}, {1, 0, 1.0 * i}}));
            }
//            detected_lines.push_back({0,0});
//
        }
        if (ImGui::Button("Load points")){
            points_in_line.clear();
            lines_3d.clear();
            detected_lines.clear();
            for (const auto local_workspace: extra_data)
            {
                std::vector<std::vector<Eigen::Vector2d>> points_in_txt = spherical_calib_utils::loadPointsListsFromTXT(local_workspace + "/lines.txt");
                // optimize points groups
                for (const auto &pls : points_in_txt) {
                    line_detector::mergepolyline pl;
                    line_detector::fitPoints(pls, sphere_img1.rows, sphere_img1.cols, -1, 1e-3, pl);
                    Eigen::Vector2d line_params = pl.initial;
                    std::cout << "line " << detected_lines.size() << " has fitting cost " << pl.cost << std::endl;
                    detected_lines.emplace_back(line_params.cast<float>());
                }
                for (int i =0;i < points_in_txt.size();i++)
                {
                    const std::string fn(local_workspace + "/line_" + std::to_string(i) + ".asc");
                    auto line = spherical_calib_utils::getLineFromASC(fn);
                    lines_3d.push_back(line);
                }
                std::move(points_in_txt.begin(), points_in_txt.end(), std::back_inserter(points_in_line));
            }
        }
        if (ImGui::Button("Optimize")){
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            Sophus::SE3d gizmo_SE3(gizmo_mat_t);
            Vector6d gizmo_se3 = gizmo_SE3.log();

            ceres::Problem problem;
            problem.AddParameterBlock(gizmo_se3.data(),6);
            for (int i =0; i < detected_lines.size();i++)
            {
                const Eigen::Vector2d line_in_image = detected_lines[i].cast<double>();
                const Vector6d line_in_3d = pluckers[i];

                ceres::CostFunction *cost_function = CostFunctor::Create(line_in_image,line_in_3d);
                ceres::LossFunction *loss = new ceres::HuberLoss(0.5);
                problem.AddResidualBlock(cost_function, loss, gizmo_se3.data());
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.line_search_direction_type = ceres::LBFGS;
//            options.line_search_sufficient_function_decrease = 1e-6;
//            options.max_line_search_step_contraction = 1e-3;
//            options.min_line_search_step_contraction =0.6;
//            options.max_num_line_search_step_size_iterations =20;
//            options.max_num_line_search_direction_restarts = 5;
//            options.line_search_sufficient_curvature_decrease = 0.9;
            options.function_tolerance = 1e-8;
            options.max_num_iterations = 500;
            options.minimizer_progress_to_stdout = true;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
            std::cout << "se3 of camera " << gizmo_se3.transpose() << std::endl;
            Sophus::SE3d gizmo_SE3_after = Sophus::SE3d::exp(gizmo_se3);
            gizmo_mat = gizmo_SE3_after.matrix().cast<float>();
        }
        ImGui::End();

        ImGui::Begin("X");
        {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            Sophus::SE3d gizmo_SE3(gizmo_mat_t);
            Vector6d gizmo_se3 = gizmo_SE3.log();
            ImGui::Text("%.3f %.3f %.3f\n%.3f %.3f %.3f ", gizmo_se3[0],gizmo_se3[1],gizmo_se3[2],gizmo_se3[3],gizmo_se3[4],gizmo_se3[5]);
        }

        if(ImGui::Button("write_img")){
            cv::imwrite(workspace+"/scr.png", sphere_img1);
        }
        if (ImGui::Button("write_calib")) {
            Eigen::Map<Eigen::Matrix4f> camera_mat_map(&camera_matrix[0][0]);
            m3d_utils::saveMat("camera_calib.txt", camera_mat_map.cast<double>());
        }
        ImGui::SameLine();
        if (ImGui::Button("load_calib")) {
            Eigen::Map<Eigen::Matrix4f> camera_mat_map(&camera_matrix[0][0]);
            camera_mat_map = m3d_utils::loadMat("camera_calib.txt").cast<float>();
        }
        ImGui::SameLine();
        if (ImGui::Button("write_pcd")) {
            pcl::PointCloud<pcl::PointXYZRGB> pcd;
            pcd.reserve(draw_buffer_cloud.size()/6);
            for (int i =0; i <draw_buffer_cloud.size(); i+=6 )
            {
                pcl::PointXYZRGB p;
                p.x= draw_buffer_cloud[i];
                p.y= draw_buffer_cloud[i+1];
                p.z= draw_buffer_cloud[i+2];
                p.r= draw_buffer_cloud[i+3]*255;
                p.g= draw_buffer_cloud[i+4]*255;
                p.b= draw_buffer_cloud[i+5]*255;
                pcd.push_back(p);
            }
            pcl::io::savePCDFile(workspace + "/scr.pcd", pcd);
        }
        ImGui::Image((void *) (intptr_t) texture_camera1.getMRendererId(), ImVec2(cols/2, rows/2));
        ImGui::End();
        // opengl drawing
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glm::mat4 proj = glm::perspective(30.f, 1.0f*width/height, 0.05f, 100.0f);
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_1 = glm::rotate(model_translate, rot_x, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(0.0f, 0.0f, 1.0f));
        glm::mat4 model_rotation_3 = glm::rotate(model_rotation_2, (float)(0.5f*M_PI), glm::vec3(-1.0f, 0.0f, 0.0f));

        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
        if (camera_gizmo) {
            ImGuizmo::Manipulate(&model_rotation_2[0][0], &proj[0][0],
                                 ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
                                 ImGuizmo::LOCAL, &camera_matrix[0][0]);
        }else
        {
            ImGuizmo::Manipulate(&model_rotation_2[0][0], &proj[0][0],
                                 ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
                                 ImGuizmo::LOCAL, &utility_matrix[0][0]);
        };

        Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());

        sphere_img1 = sphere_img_color.clone();
        if(!draw_det) {
            for (int i = 0; i < lines_3d.size(); i++) {
                const auto &p = lines_3d[i];
                const auto &c = colors[i];
                Vector6d plucker_local = getPluckerLineTransformer<double>(gizmo_mat_t.inverse()) * pluckers[i];
                const float z_dist = plucker_local.block<3, 1>(0, 0).norm();
                const float inclination = std::acos(plucker_local.z() / z_dist);
                const float azimuth = std::atan2(plucker_local.x(), plucker_local.y());

                sphere_img1 = drawGreatCircle(sphere_img1, azimuth, inclination,cv::Vec3b(c.z() * 255, c.y() * 255, c.x() * 255));

            }

        }else {
            for (int i = 0; i < detected_lines.size(); i++) {
                const auto &c = colors[i];
                sphere_img1 = drawGreatCircle(sphere_img1, detected_lines[i][0], detected_lines[i][1],
                                              cv::Vec3b(c.z() * 255, c.y() * 255, c.x() * 255));
            }
            for (int i = 0; i < points_in_line.size(); i++) {
                const auto &c = colors[i];
                for (int j=0; j < points_in_line[i].size(); j++)
                {
                    cv::Vec2i xy(points_in_line[i][j].x(),points_in_line[i][j].y());
                    cv::drawMarker(sphere_img1, xy, cv::Vec3b(c.z() * 255, c.y() * 255, c.x() * 255) ,cv::MARKER_STAR, 40, 2 );
                }
            }
        }


        texture_camera1.update(sphere_img1);

        GLCall(glLineWidth(2));
        shader.Bind(); // bind shader to apply uniform
        //shader.setUniformMat4f("u_MVP", proj * model_rotation_2*camera_matrix);
        //renderer.Draw(va_co, ib_co, shader, GL_LINES);
        shader.setUniformMat4f("u_MVP", proj * model_rotation_2);
        renderer.Draw(va_lines,ib_lines,shader, GL_LINES);
        GLCall(glPointSize(1));
        renderer.Draw(va_pointloud,ib_pointcloud,shader, GL_POINTS);


        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
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
m3d_utils::ladybug_camera_calibration ladybug_calibration;
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
    const Vector6d line_in_scene; //
    const Eigen::Vector3d moment_observed;
    const Eigen::Matrix4d local_extrinsic;
    CostFunctor(const Eigen::Vector3d& moment_observed,
                const Vector6d& line_in_scene, const Eigen::Matrix4d& local_extrinsic) :
            moment_observed(moment_observed), line_in_scene(line_in_scene),
            local_extrinsic(local_extrinsic){

    }

    template <typename T>
    bool operator()(const T* const posetangent,
                    T* residuals) const {
        Sophus::SE3<T> params1 = getSEFromParams<T>(posetangent);

//        Vector6d plucker_local =
//                getPluckerLineTransformer<double>((gizmo_mat_t * cam_extrinsic).inverse()) * pluckers[i];
//
        Eigen::Matrix<T,4,4> params1_inv = (params1.matrix()*local_extrinsic.cast<T>()).inverse();
        Eigen::Matrix<T,6,1> plucker_local = getPluckerLineTransformer<T>(params1_inv) * line_in_scene;
        T z_dist = plucker_local.template block<3, 1>(0, 0).norm();

        Eigen::Matrix<T,3,1> plucker_moment = plucker_local.template block<3, 1>(0, 0);
        Eigen::Matrix<T,3,1> plucker_moment_norm = plucker_local.template block<3, 1>(0, 0)/z_dist;


        Eigen::Matrix<T,3,1> plucker_dir_norm = plucker_local.template block<3, 1>(3, 0);

        auto moment_observed_T = moment_observed.cast<T>();
        Eigen::Matrix<T,3,1> l_moment = moment_observed_T.cross(plucker_moment);
        //Eigen::Matrix<T,3,1> l_dir = moment_observed_T.cross(plucker_dir_norm);

        residuals[0] = l_moment.squaredNorm();
        residuals[1] = params1.translation().x()*params1.translation().x()+params1.translation().y()*params1.translation().y();
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& moment_observed, const Vector6d& line_in_scene,
                                       const Eigen::Matrix4d& local_extrinsic) {
        return (new ceres::AutoDiffCostFunction<CostFunctor, 2, 6>(
                new CostFunctor(moment_observed, line_in_scene, local_extrinsic)));
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


cv::Mat& drawPlane(cv::Mat &sphere_img, Eigen::Vector3d normal , cv::Vec3b color,
                   double cameraFocalLen, double cameraCenterX, double cameraCenterY )
{

    normal = normal /normal.norm();
    cv::Vec2d  beg;
    cv::Vec2d  end;

    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0,0) = cameraFocalLen;
    K(1,1) = cameraFocalLen;
    K(2,0) = -cameraCenterX*cameraFocalLen;
    K(2,1) = -cameraCenterY*cameraFocalLen;
    K(2,2) = cameraFocalLen*cameraFocalLen;

//    std::cout << "======" << std::endl;
//    std::cout << K << std::endl;
    Eigen::Vector3d lc = K * normal;
    //lc = lc / (std::sqrt(lc[0]*lc[0]+lc[1]*lc[1]));
    //lc = lc / (std::sqrt(lc[0]*lc[0]+lc[1]*lc[1]));
    lc = lc / lc.norm();


    beg[0] = 0;
    beg[1] = (-lc[0]*0 -lc[2])/lc[1];
//
    end[0] =  sphere_img.cols;
    end[1] = (-lc[0]*sphere_img.cols -lc[2])/lc[1];
//
    cv::line(sphere_img, cv::Vec2i(beg), cv::Vec2i(end), color,3);


    return sphere_img;
}

bool updateDrawingBufferWithColor(std::vector<float> &draw_buffer_cloud, std::vector<unsigned int> &draw_buffer_cloud_indices,
                                  const pcl::PointCloud<pcl::PointXYZRGB> &pc,
                                  const std::vector<cv::Mat> &images, const m3d_utils::ladybug_camera_calibration &ladybug_calibration,
                                  const Eigen::Matrix4d& calibration_camera){
    draw_buffer_cloud.clear();
    draw_buffer_cloud_indices.clear();

    std::cout << "calibration_camera " << std::endl;
    std::cout << calibration_camera << std::endl;
//    const int camera_id = 0;
//    auto camera_extrinsic_loc=ladybug_calibration.getCameraExtrinsic()[camera_id].cast<double>();
//    double camera1_FocalLen = ladybug_calibration.getCameraFocal()[camera_id];
//    double camera1_CameraCenterX =  ladybug_calibration.getCameraCenter()[camera_id].x();
//    double camera1_CameraCenterY =  ladybug_calibration.getCameraCenter()[camera_id].y();

    //camera_extrinsic_loc = camera_extrinsic_loc.inverse();
    for (const auto p: pc)
    {
        if (!images.empty()) {

            //Eigen::Matrix<double,1,4> pt_cam = camera_extrinsic_loc.inverse()*sphere_odometry.inverse()*pt.transpose();
            float distance_to_center_min = std::numeric_limits<float>::max();
            cv::Vec3b best_color;
            for (int camera_id = 0; camera_id < images.size(); camera_id++) {
                Eigen::Matrix<double, 1, 4> pt{p.x, p.y, p.z, 1.0};
                Eigen::Matrix<double, 4, 4> ladybug_loc_calib = ladybug_calibration.getCameraExtrinsic()[camera_id].cast<double>();
                Eigen::Matrix<double, 1, 4> pt_cam =
                        (calibration_camera * ladybug_loc_calib).inverse() * pt.transpose();

                float camera_FocalLen = ladybug_calibration.getCameraFocal()[camera_id];
                const auto &camera_center = ladybug_calibration.getCameraCenter()[camera_id];
                const double xx = camera_FocalLen * (pt_cam.x() / pt_cam.z());
                const double yy = camera_FocalLen * (pt_cam.y() / pt_cam.z());
                double distance_to_center = xx * xx + yy * yy;
                const double xx2 = xx + camera_center.x();
                const double yy2 = yy + camera_center.y();
                const auto &img = images[camera_id];
                if (distance_to_center < distance_to_center_min && pt_cam.z() > 0 && xx2 > 0 && yy2 > 0 &&
                    xx2 < img.cols && yy2 < img.rows) {
                    distance_to_center_min = distance_to_center;
                    best_color = img.at<cv::Vec3b>(yy2, xx2);
                }


            }
            if (distance_to_center_min != std::numeric_limits<float>::max()) {
                draw_buffer_cloud.insert(draw_buffer_cloud.end(), {p.x, p.y, p.z});
                draw_buffer_cloud.insert(draw_buffer_cloud.end(),
                                         {(1.0f * best_color[2]) / 256.0f, (1.0f * best_color[1]) / 256.0f,
                                          (1.0f * best_color[0]) / 256.0f});
                draw_buffer_cloud_indices.push_back(draw_buffer_cloud_indices.size());

            }
        }
        else {
            draw_buffer_cloud.insert(draw_buffer_cloud.end(), { p.x,p.y,p.z });
            draw_buffer_cloud.insert(draw_buffer_cloud.end(), {1.0f * p.r / 255, 1.0f * p.g / 255, 1.0f * p.g / 255});
            draw_buffer_cloud_indices.push_back(draw_buffer_cloud_indices.size());
        }
    }
    std::cout << "draw_buffer_cloud_indices " << draw_buffer_cloud_indices.size()<<std::endl;
}

cv::Mat creteView(const std::vector<cv::Mat>& images)
{
    cv::Mat result;
    cv::hconcat(images, result );
    return result;
}
struct line_in_image{
    int image_id;
    std::vector<Eigen::Vector2f> points_in_line;
    Eigen::Vector3d normal;
    bool use {true};

};
Eigen::Matrix4f getIntialCalib()
{
    Sophus::Vector6f initial_se3;
    initial_se3.setZero();
    initial_se3[5] = -1.9f;
    auto const k = Sophus::SE3f::exp(initial_se3);
    return k.matrix();
}
int main(int argc, char **argv) {

    std::string workspace(argv[1]);
    std::cout << "workspace " << workspace << std::endl;
    std::vector<std::string> extra_data{
            workspace
    };

    std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> lines_3d{};
    int gizmed_line_id = -1;

    std::vector<Eigen::Vector3f> colors{{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,1,0},{1,1,0},{1,1,0}};

    std::vector<line_in_image> detected_lines;

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
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::PointCloud<pcl::PointXYZINormal> pc_raw;

    pcl::io::loadPCDFile(workspace+"/1635402399_pointcloud_raw.pcd", pc_raw);

    // apply calibration
    Eigen::Matrix4f bighead_calib = m3d_utils::loadMat("calib.txt").cast<float>();
    // gety ladybug calibration
    ladybug_calibration.loadConfig(workspace+"/camera_params.ini");


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
    //pcl::io::savePCDFile(workspace+"/pointcloud_transformed.pcd", pc);
    //pcl::io::loadPCDFile("/tmp/cloud.pcd", pc);

    //pcl::io::loadPCDFile("/mnt/540C28560C283580/dane_plecak/tests/front/25/cloud.pcd", pc);

    updateDrawingBufferWithColor(draw_buffer_cloud, draw_buffer_cloud_indices, pc, std::vector<cv::Mat>(),
            m3d_utils::ladybug_camera_calibration(), Eigen::Matrix4d());
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

    Eigen::Map<Eigen::Matrix4f> camera_mat_map(&camera_matrix[0][0]);
    camera_mat_map = getIntialCalib();

    std::vector<cv::Mat> camera_images;
    for (int i =0; i < 6; i++)
    {
        const std::string img_fn = workspace+"/1635402399_ladybugPostProcess-rectified"+std::to_string(i)+"-0.jpg";
        std::cout << "img_fn " << img_fn << std::endl;
        cv::Mat sphere_img_color = cv::imread(img_fn);
        cv::resize(sphere_img_color,sphere_img_color,cv::Size(2448,2048));
        camera_images.push_back(sphere_img_color);
    }


    bool draw_det {true};
    Texture texture_camera1(creteView(camera_images));
    bool camera_gizmo = false;
    bool gl_dirty = true;
    cv::Mat gl_image;
    while (!glfwWindowShouldClose(window)) {
        draw_buffer_line.clear();
        draw_line_indices.clear();
        for (int i = 0; i < lines_3d.size(); i++) {
            const auto &p = lines_3d[i];
            const auto &c = colors[i];
            addLineToBuffer(draw_buffer_line, draw_line_indices, p.first, p.second, c);
        }
        vb_lines.update(draw_buffer_line.data(), draw_buffer_line.size() * sizeof(float));
        ib_lines.update(draw_line_indices.data(), draw_line_indices.size());

        std::vector<Vector6d> pluckers;
        for (const auto &p : lines_3d) {
            pluckers.push_back(getPlucker(p.first, p.second));
        }

        GLCall(glEnable(GL_DEPTH_TEST));
        GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();

        ImGui::NewFrame();

        std::stringstream oss;
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

        ImGui::Begin("OBS");
        ImGui::Checkbox("camera_gizmo", &camera_gizmo);
        if (ImGui::Button("reset")){
            Eigen::Map<Eigen::Matrix4f> mat_map(&camera_matrix[0][0]);
            mat_map = getIntialCalib();
        }
        if (ImGui::Button("update")) {
            gl_dirty = true;
        }


        ImGui::Checkbox("draw_detection", &draw_det);
        for (int i = 0; i < detected_lines.size(); i++) {
            if (ImGui::Button(("set_g" + std::to_string(i)).c_str())) {
                Eigen::Map<Eigen::Matrix4f> utility_matrix_map(&utility_matrix[0][0]);
                Eigen::Matrix4d utility_matrix_or = m3d_utils::orthogonize(utility_matrix_map.cast<double>());
                //utility_matrix_or = utility_matrix_or.inverse();
                lines_3d[i].first = Eigen::Affine3d(utility_matrix_or) * Eigen::Vector3d({-1, 0, 0});
                lines_3d[i].second = Eigen::Affine3d(utility_matrix_or) * Eigen::Vector3d({1, 0, 0});
            }
            ImGui::SameLine();
            if (ImGui::Button(("get_g" + std::to_string(i)).c_str())) {
                Eigen::Map<Eigen::Matrix4f> utility_matrix_map(&utility_matrix[0][0]);

                Eigen::Matrix4d mat(Eigen::Matrix4d::Identity());
                Eigen::Vector3d m = lines_3d[i].first - lines_3d[i].second;
                mat(0, 3) = lines_3d[i].first.x() - m.x() / 2;
                mat(1, 3) = lines_3d[i].first.y() - m.y() / 2;
                mat(2, 3) = lines_3d[i].first.z() - m.z() / 2;

                mat.block<3, 1>(0, 0) = m / m.norm();
                mat.block<3, 1>(0, 2) = Eigen::Vector3d{0, 0, 1};
                mat.block<3, 1>(0, 1) = mat.block<3, 1>(0, 2).cross(mat.block<3, 1>(0, 0));
                utility_matrix_map = mat.cast<float>();
                //utility_matrix_or = m3d_utils::orthogonize(utility_matrix_map.cast<double>());
                //utility_matrix_or = utility_matrix_or.inverse();
            }
            ImGui::SameLine();
            if (ImGui::Button(("get_from_asc" + std::to_string(i)).c_str())) {
                const std::string fn(workspace + "/line_" + std::to_string(i) + ".asc");
                    auto line = spherical_calib_utils::getLineFromASC(fn);
                    lines_3d[i] = line;
                gl_dirty = true;
            }
            ImGui::SameLine();
            if (ImGui::Button(("get_from_txt" + std::to_string(i)).c_str())) {
                const std::string fn(workspace + "/line_" + std::to_string(i) + ".txt");
                line_in_image line;
                std::ifstream ifss(fn);
                ifss >> line.image_id;
                while(!ifss.eof()){
                    Eigen::Vector2f p;
                    ifss >> p.x();
                    ifss >> p.y();
                    line.points_in_line.push_back(p);
                }
                ceres::Problem problem;
                Eigen::Vector3d plane_normal{1,1,1};
                problem.AddParameterBlock(plane_normal.data(),3);
                for (const auto p2d :line.points_in_line){
                    const double focal_len = ladybug_calibration.getCameraFocal()[line.image_id];
                    const Eigen::Vector2f proj_center = ladybug_calibration.getCameraCenter()[line.image_id];

                    ceres::CostFunction *cost_function = bigUnitCalib::
                            OptimizePlaneNormalOnCameraImage::Create(p2d,proj_center, focal_len);
                    ceres::LossFunction *loss = nullptr;
                    problem.AddResidualBlock(cost_function, loss, plane_normal.data());
                }
                ceres::Solver::Options options;
                options.max_num_iterations = 50;
                options.minimizer_progress_to_stdout = true;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);

                std::cout << summary.FullReport() << "\n";
                line.normal = plane_normal/plane_normal.norm();
                detected_lines[i] = line;
                gl_dirty = true;
            }
            ImGui::SameLine();
            ImGui::Checkbox(("use_" + std::to_string(i)).c_str(), &detected_lines[i].use);
        }
        if (ImGui::Button("add")) {
            detected_lines.resize(detected_lines.size()+1);
            lines_3d.push_back(std::pair<Eigen::Vector3d, Eigen::Vector3d>({{0, 0, 0},
                                                                            {1, 0, 0}}));
            gl_dirty = true;
        };
        ImGui::SameLine();

//        if (ImGui::Button("load")) {
//            Eigen::Matrix4f m;
//            loadLines(detected_lines, lines_3d, m, workspace);
//            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
//            gizmo_mat = m;
//        }
//        ImGui::SameLine();
//        if (ImGui::Button("save")) {
//            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
//            writeLines(detected_lines, lines_3d, gizmo_mat, workspace);
//        }
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
            gl_dirty = true;
        }
        if (ImGui::Button("color")) {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());

            updateDrawingBufferWithColor(draw_buffer_cloud, draw_buffer_cloud_indices, pc, camera_images,
                                         ladybug_calibration, gizmo_mat_t);

//            updateDrawingBufferWithColor(draw_buffer_cloud, draw_buffer_cloud_indices, pc, sphere_img_color,
//                                         gizmo_mat_t);

            vb_pointcloud.update(draw_buffer_cloud.data(), draw_buffer_cloud.size() * sizeof(float));
            ib_pointcloud.update(draw_buffer_cloud_indices.data(), draw_buffer_cloud_indices.size());
        }
        ImGui::SameLine();
        if (ImGui::Button("uncolor")) {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            updateDrawingBufferWithColor(draw_buffer_cloud, draw_buffer_cloud_indices, pc, std::vector<cv::Mat>(),
                                         m3d_utils::ladybug_camera_calibration(), Eigen::Matrix4d());

            vb_pointcloud.update(draw_buffer_cloud.data(), draw_buffer_cloud.size() * sizeof(float));
            ib_pointcloud.update(draw_buffer_cloud_indices.data(), draw_buffer_cloud_indices.size());
        }

        if (ImGui::Button("Optimize")) {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            Sophus::SE3d gizmo_SE3(gizmo_mat_t);
            Vector6d gizmo_se3 = gizmo_SE3.log();

            ceres::Problem problem;
            problem.AddParameterBlock(gizmo_se3.data(), 6);
            for (int i = 0; i < detected_lines.size(); i++) {
                if (!detected_lines[i].use) continue;
                const Eigen::Vector3d line_in_image = detected_lines[i].normal;
                const int camera_id = detected_lines[i].image_id;
                ASSERT(camera_id < ladybug_calibration.getCameraExtrinsic().size());
                const Eigen::Matrix4d camera = ladybug_calibration.getCameraExtrinsic()[camera_id].cast<double>();
                const Vector6d line_in_3d = pluckers[i];

                ceres::CostFunction *cost_function = CostFunctor::Create(line_in_image, line_in_3d, camera);
                ceres::LossFunction *loss = nullptr;// new ceres::HuberLoss(0.01);
                problem.AddResidualBlock(cost_function, loss, gizmo_se3.data());
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.line_search_direction_type = ceres::STEEPEST_DESCENT;
//            options.line_search_sufficient_function_decrease = 1e-6;
//            options.max_line_search_step_contraction = 1e-3;
//            options.min_line_search_step_contraction =0.6;
//            options.max_num_line_search_step_size_iterations =20;
//            options.max_num_line_search_direction_restarts = 5;
//            options.line_search_sufficient_curvature_decrease = 0.9;
            options.function_tolerance = 1e-12;
            options.max_num_iterations = 5000;
            options.minimizer_progress_to_stdout = true;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
            std::cout << "se3 of camera " << gizmo_se3.transpose() << std::endl;
            Sophus::SE3d gizmo_SE3_after = Sophus::SE3d::exp(gizmo_se3);
            gizmo_mat = gizmo_SE3_after.matrix().cast<float>();
            gl_dirty = true;
        }

        ImGui::End();

        ImGui::Begin("X");
        {
            Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
            Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());
            Sophus::SE3d gizmo_SE3(gizmo_mat_t);
            Vector6d gizmo_se3 = gizmo_SE3.log();
            ImGui::Text("%.3f %.3f %.3f\n%.3f %.3f %.3f ", gizmo_se3[0], gizmo_se3[1], gizmo_se3[2], gizmo_se3[3],
                        gizmo_se3[4], gizmo_se3[5]);
        }
        if (ImGui::Button("refresh_image")){
            gl_dirty = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("write_img")) {
            cv::imwrite(workspace + "/scr.jpg", gl_image);
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

        if (ImGui::Button("write_calib")) {
            m3d_utils::saveMat("camera_calib.txt", camera_mat_map.cast<double>());
        }
        ImGui::SameLine();
        if (ImGui::Button("load_calib")) {
            camera_mat_map = m3d_utils::loadMat("camera_calib.txt").cast<float>();
        }
        ImGui::Image((void *) (intptr_t) texture_camera1.getMRendererId(), ImVec2(2000,600));
        ImGui::End();
        // opengl drawing
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glm::mat4 proj = glm::perspective(30.f, 1.0f * width / height, 0.05f, 100.0f);
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_1 = glm::rotate(model_translate, rot_x, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(0.0f, 0.0f, 1.0f));
        glm::mat4 model_rotation_3 = glm::rotate(model_rotation_2, (float) (0.5f * M_PI), glm::vec3(-1.0f, 0.0f, 0.0f));

        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuiIO &io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        Eigen::Map<Eigen::Matrix4f> gizmo_mat(&camera_matrix[0][0]);
        if (camera_gizmo) {
            ImGuizmo::Manipulate(&model_rotation_2[0][0], &proj[0][0],
                                 ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
                                 ImGuizmo::LOCAL, &camera_matrix[0][0]);
        } else {
            ImGuizmo::Manipulate(&model_rotation_2[0][0], &proj[0][0],
                                 ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
                                 ImGuizmo::LOCAL, &utility_matrix[0][0]);
        };

        Eigen::Matrix4d gizmo_mat_t = m3d_utils::orthogonize(gizmo_mat.cast<double>());

//        perspective_img1 = sphere_img_color.clone();

        if (gl_dirty) {
            if (!draw_det) {
                std::vector<cv::Mat> camera_images_cpy;
                for (int i = 0; i < camera_images.size(); i++) {
                    camera_images_cpy.push_back(camera_images[i].clone());
                }
                for (int i = 0; i < lines_3d.size(); i++) {
                    const auto &p = lines_3d[i];
                    const auto &c = colors[i];
                    for (int camera_id = 0; camera_id < camera_images.size(); camera_id++) {
                        const auto cam_extrinsic = ladybug_calibration.getCameraExtrinsic()[camera_id].cast<double>();
                        Vector6d plucker_local =
                                getPluckerLineTransformer<double>((gizmo_mat_t * cam_extrinsic).inverse()) *
                                pluckers[i];
                        const float z_dist = plucker_local.block<3, 1>(0, 0).norm();
                        Eigen::Affine3d k(gizmo_mat_t * cam_extrinsic);
                        std::pair<Eigen::Vector3d, Eigen::Vector3d> line3d_t = lines_3d[i];
                        line3d_t.first = k.inverse() * lines_3d[i].first;
                        line3d_t.second = k.inverse() * lines_3d[i].second;

                        const double foc_l = ladybug_calibration.getCameraFocal()[camera_id];
                        const auto center = ladybug_calibration.getCameraCenter()[camera_id];
                        camera_images_cpy[camera_id] = drawPlane(camera_images_cpy[camera_id], plucker_local.head<3>(),
                                                                 cv::Vec3b(c.z() * 255, c.y() * 255, c.x() * 255),
                                                                 foc_l, center.x(), center.y());
                        //perspective_img1 = drawPlane(perspective_img1, line3d_t, cv::Vec3b(c.z() * 255, c.y() * 255, c.x() * 255));
                    }
                }

                gl_image = creteView(camera_images_cpy);
                texture_camera1.update(gl_image);
            } else {
                std::vector<cv::Mat> camera_images_cpy;
                for (int i = 0; i < camera_images.size(); i++) {
                    camera_images_cpy.push_back(camera_images[i].clone());
                }


                for (int i = 0; i < detected_lines.size(); i++) {
                    const auto &c = colors[i];
                    const auto &line = detected_lines[i];
                    const int camera_id = line.image_id;

                    ASSERT(camera_id < camera_images_cpy.size());
                    for (const auto &p :line.points_in_line) {
                        cv::Point pcv(p.y(), p.x());
                        cv::drawMarker(camera_images_cpy[camera_id], pcv,
                                       cv::Vec3b(c.z() * 255, c.y() * 255, c.x() * 255),
                                       cv::MARKER_STAR, 40, 2);
                    }
                    const double foc_l = ladybug_calibration.getCameraFocal()[camera_id];
                    const auto center = ladybug_calibration.getCameraCenter()[camera_id];

                    camera_images_cpy[camera_id] = drawPlane(camera_images_cpy[camera_id], line.normal,
                                                             cv::Vec3b(c.z() * 255, c.y() * 255, c.x() * 255), foc_l,
                                                             center.x(), center.y());
                }
                gl_image = creteView(camera_images_cpy);
                texture_camera1.update(gl_image);
            }
            gl_dirty = false;
        }



        GLCall(glLineWidth(2));
        shader.Bind(); // bind shader to apply uniform
        //shader.setUniformMat4f("u_MVP", proj * model_rotation_2*camera_matrix);
        //renderer.Draw(va_co, ib_co, shader, GL_LINES);
        shader.setUniformMat4f("u_MVP", proj * model_rotation_2);
        renderer.Draw(va_lines, ib_lines, shader, GL_LINES);
        GLCall(glPointSize(2));
        renderer.Draw(va_pointloud, ib_pointcloud, shader, GL_POINTS);

        for (int i = 0; i < ladybug_calibration.getCameraExtrinsic().size(); i++)
        {
            glm::mat4 camera= glm::mat4(1.0f);
            glm::mat4 cam_scale = glm::scale(glm::mat4(1.0f), glm::vec3(0.1,0.1,0.1));
            Eigen::Map<Eigen::Matrix4f> camera_e(&camera[0][0]);
            camera_e = ladybug_calibration.getCameraExtrinsic()[i].cast<float>();
            shader.setUniformMat4f("u_MVP", proj * model_rotation_2* camera_matrix *camera * cam_scale);
            renderer.Draw(va_co, ib_co, shader, GL_LINES);
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
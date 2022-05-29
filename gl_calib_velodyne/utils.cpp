#include "utils.h"
#include <random>

Eigen::Matrix4d m3d_utils::loadMat(const std::string& fn){
    Eigen::Matrix4d m;
    std::ifstream ifss(fn);
    for (int i =0; i < 16; i++) {
        ifss >> m.data()[i];
    }
    ifss.close();
    return m.transpose();
}

Sophus::Vector6d m3d_utils::loadVec6(const std::string& fn){
    Sophus::Vector6d m;
    std::ifstream ifss(fn);
    for (int i =0; i < 6; i++) {
        ifss >> m.data()[i];
    }
    ifss.close();
    return m.transpose();
}

void m3d_utils::saveMat(const std::string& fn, const Eigen::Matrix4d& mat){
    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "", "");
    std::ofstream fmt_ofs (fn);
    fmt_ofs << mat.format(HeavyFmt);
    fmt_ofs.close();
}

void m3d_utils::saveVec6(const std::string& fn, const Sophus::Vector6d & mat){
    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "", "");
    std::ofstream fmt_ofs (fn);
    fmt_ofs << mat.transpose().format(HeavyFmt);
    fmt_ofs.close();
}

std::pair<double,double> m3d_utils::findVariance(const std::vector<double> &data){
    double mean = 0.;
    double var = 0.;

    for (double e:data){
        mean+=e;
    }
    mean = mean/data.size();

    for (double e:data){
        var += (mean-e)*(mean-e);
    }
    var = var/data.size();
    return std::pair<double,double>(mean,var);
}

double getMSE(const cv::Mat& I1, const cv::Mat& I2)
{
    cv::Mat s1;
    cv::absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2
    cv::Scalar s = sum(s1);         // sum elements per channel
    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels
    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double  mse =sse /(double)(I1.channels() * I1.total());
        return mse;
        // Instead of returning MSE, the tutorial code returned PSNR (below).
        //double psnr = 10.0*log10((255*255)/mse);
        //return psnr;
    }
}

m3d_utils::TestPlane::TestPlane (std::string T)
{
    float* b = &matrix[0][0];
    std::stringstream ss(T);
    for (int i = 0; i < 16; i ++ )
    {
    ss >> b[i];
    }
    for (int i = 0; i < 16; i ++ )
    {
    std::cout <<  b[i];
    }
    std::cout <<  std::endl;
}

Eigen::Affine3d m3d_utils::orthogonize(const Eigen::Affine3d& p )
{
    Eigen::Matrix4d ret = p.matrix();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(ret.block<3,3>(0,0), Eigen::ComputeFullU | Eigen::ComputeFullV);
    double d = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    Eigen::Matrix3d diag = Eigen::Matrix3d::Identity() * d;
    ret.block<3,3>(0,0) = svd.matrixU() * diag * svd.matrixV().transpose();
    return Eigen::Affine3d (ret);
//    Eigen::Affine3d pose_orto(Eigen::Affine3d::Identity());
//    Eigen::Quaterniond q1(p.matrix().block<3,3>(0,0)); q1.normalize();
//    pose_orto.translate(p.matrix().block<3,1>(0,3));
//    pose_orto.rotate(q1);
//    return pose_orto;
}

Eigen::Matrix4d m3d_utils::orthogonize(const Eigen::Matrix4d& p )
{
    Eigen::Affine3d pose_orto(Eigen::Affine3d::Identity());
    Eigen::Quaterniond q1(p.block<3,3>(0,0)); q1.normalize();
    pose_orto.translate(p.block<3,1>(0,3));
    pose_orto.rotate(q1);
    return pose_orto.matrix();
}

boost::property_tree::ptree m3d_utils::TestPlane::serialize() const
{
    boost::property_tree::ptree pt;
    const float* cb = &matrix[0][0];
    std::stringstream ss;
    for (int i =0; i < 16; i++)
    {
        ss << cb[i] << " ";
    }
    pt.put("T",ss.str());  class ladybug_camera_calibration{
    public:
        void loadConfig(const std::string &fn);
        void loadCfgFromString(const std::string &data);
        const std::vector<Eigen::Matrix4f> &getCameraExtrinsic() const;
        const std::vector<float> &getCameraFocal() const;
        const std::vector<Eigen::Vector2f> &getCameraCenter() const;

    private:
        static Eigen::Matrix4d makeTransformation( const double rotX, const double rotY, const double rotZ, const double transX, const double transY, const double transZ);
        std::vector<Eigen::Matrix4f> camera_extrinsic;
        std::vector<float> camera_focal;
        std::vector<Eigen::Vector2f> camera_center;

    };
    return pt;
}
std::vector<Eigen::Vector2d> m3d_utils::detectMarkerInImage(const cv::Mat &input_image, const cv::Mat& marker)
{
    const int s_upscale = 1;
    const int s_border  = 10;
    std::vector<Eigen::Vector2d> ret;
    // to gray
    cv::Mat input_gray;
    cv::Mat pattern_gray;
    cv::cvtColor(input_image,input_gray, cv::COLOR_BGR2GRAY );
    cv::cvtColor(marker,pattern_gray, cv::COLOR_BGR2GRAY );
    cv::resize(pattern_gray,pattern_gray,cv::Size(0,0),s_upscale,s_upscale);
    cv::Mat input_gray_canny;
    cv::GaussianBlur(input_gray,input_gray, cv::Size(3,3),1 );
    cv::Canny(input_gray,input_gray_canny,150,255);

    cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                         cv::Size( 2*3 + 1, 2*3+1 ),
                                         cv::Point( 3, 3 ) );

    cv::dilate(input_gray_canny,input_gray_canny,element);
    cv::erode(input_gray_canny,input_gray_canny,element);

    cv::imwrite("/tmp/dbg_03_canny.png", input_gray_canny);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(input_gray_canny,contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    cv::Mat drawing = cv::Mat::zeros( input_gray_canny.size(), CV_8UC3 );
    static std::default_random_engine rng(0);
    std::uniform_int_distribution<int> dist(0, 256);


    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color(1*dist(rng),1*dist(rng), 1*dist(rng));
        drawContours( drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
    }
    cv::imwrite("/tmp/dbg_04_contours.png", drawing);


    cv::RotatedRect best_rr;
    float best_cost = std::numeric_limits<float>::max();
    cv::Mat best_img;
    cv::Mat best_homography;
    cv::Mat dbg_polylines;
    input_image.copyTo(dbg_polylines);
    for( size_t i = 0; i< contours.size(); i++ )
    {
        std::vector<cv::Point> approx;
        double peri = cv::arcLength(contours[i], true);
        cv::approxPolyDP(contours[i],approx, 0.02 * peri, true);
        std::cout << " i : " << i << " " << peri << " " << approx.size() << std::endl;
        std::vector<cv::Point2f> corners {
            {0.f,0.f},{1.f*pattern_gray.cols,0.f},
            {1.f*pattern_gray.cols,1.f*pattern_gray.rows},{0.f,1.f*pattern_gray.rows}};
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);


        cv::Scalar color(1*dist(rng),1*dist(rng), 1*dist(rng));
        cv::polylines( dbg_polylines, approx,true,  color, 2, cv::LINE_8 );

        if (rect.size.width < 50 || rect.size.height < 50 ||  approx.size() != 4) continue;
        std::vector<cv::Point2f> pts;
        for (int i =0; i < approx.size(); i++)
        {
            pts.push_back(approx[i]);
        }

        cv::Mat H_c = cv::findHomography(pts, corners);
        cv::Mat candidate;
        cv::Mat candidate_r;

        cv::warpPerspective(input_gray, candidate,  H_c, cv::Size(pattern_gray.rows,pattern_gray.cols));

        float mse_h = getMSE(candidate, pattern_gray);
        if (mse_h< best_cost)
        {
            best_cost = mse_h;
            best_rr=rect;
            best_img = candidate;
            best_homography = H_c;
        }

        cv::rotate(candidate, candidate_r, cv::ROTATE_90_CLOCKWISE);
        float mse_v = getMSE(candidate, pattern_gray);
        if (mse_v< best_cost)
        {
            best_cost = mse_v;
            best_rr=rect;
            best_img = candidate_r;
            best_homography = H_c;
        }

        cv::imwrite("/tmp/dbg_05_candidate"+std::to_string(i)+".png",candidate);
        std::cout << "i"<< i << " " << std::min(mse_h, mse_v)<<std::endl;
    }
    cv::imwrite("/tmp/dbg_06_polyDP.png",dbg_polylines);
    cv::imwrite("/tmp/dbg_07_best_img.png",best_img);
    cv::Mat best_img_th;
    //threshold
    cv::threshold(best_img, best_img_th,255/4,255, cv::ThresholdTypes::THRESH_BINARY);

    cv::imwrite("/tmp/dbg_08_best_img_th.png",best_img_th);

    std::vector<std::vector<cv::Vec2f>> patches;
    patches.resize(4);

    const cv::Vec2i middle ( best_img_th.cols/2,  best_img_th.rows/2);
    std::cout << middle << std::endl;

    const int border = s_upscale * s_border;
    for (int y=0; y < best_img_th.rows; y++ )
    {
        uchar* pixel = best_img_th.ptr<uchar>(y);
        for (int x=0; x < best_img_th.cols; x++ )
        {
            uchar pixel_val = pixel[x];
            if (pixel_val == 255) {
                int class_id = 0;
                if (x > middle[0] && y > middle[1]) class_id = 3;
                if (x > middle[0] && y < middle[1]) class_id = 0;
                if (x < middle[0] && y > middle[1]) class_id = 1;
                if (x < middle[0] && y < middle[1]) class_id = 2;
                if (x> border && y > border && y < best_img_th.rows -border &&  x < best_img_th.cols -border) {
                    patches[class_id].push_back(cv::Vec2f(x, y));
                }
            }
        }
    }
    std::vector<cv::Vec2f> centroids; centroids.resize(4);
    cv::Mat classes( best_img_th.size(), CV_8UC3, cv::Scalar(0,0,0));
    for (int i =0; i < 4; i++)
    {
        cv::Vec2f avg = cv::Vec2f(0.f,0.f);
        for (const auto &f : patches[i])
        {
            cv::Scalar color (0,0,0);
            if (i==0) color = cv::Scalar(255,0,0);
            if (i==1) color = cv::Scalar(0,255,0);
            if (i==2) color = cv::Scalar(0,0,255);
            if (i==3) color = cv::Scalar(255,255,0);
            avg+=f;
            cv::circle(classes,cv::Vec2i(f),1,color,0);
        }
        centroids[i][0] = avg[0] / float(patches[i].size());
        centroids[i][1] = avg[1] / float(patches[i].size());
        cv::circle(classes, cv::Vec2i(centroids[i]),20,cv::Scalar (255,255,255),5);
        std::cout <<" - > " << centroids[i] <<std::endl;
    }
    cv::imwrite("/tmp/dbg_09_classes.png",classes);

    std::vector<cv::Vec2f> input_image_centroids;
    cv::perspectiveTransform(centroids, input_image_centroids,best_homography.inv());
    std::vector<std::pair<cv::Vec2f,float>> input_image_centroids_arranged;
    for (int i =0; i < 4; i++) {
        cv::Vec2f p = input_image_centroids[i];
        float ang = std::atan2(p[1]-best_rr.center.y,p[0]-best_rr.center.x)-M_PI;
        input_image_centroids_arranged.push_back(std::make_pair(p,ang));
    }
    std::sort(input_image_centroids_arranged.begin(),input_image_centroids_arranged.end(),
              [](const std::pair<cv::Vec2f,float> &p1, const std::pair<cv::Vec2f,float> &p2 ){
        return p1.second > p2.second;
    });
    cv::Mat input_image_centroids_dbg;

    input_image.copyTo(input_image_centroids_dbg);

    std::rotate(input_image_centroids_arranged.begin(),
                input_image_centroids_arranged.begin()+1,
                input_image_centroids_arranged.end());

    for (int i =0; i < 4; i++)
    {
        cv::Vec2f p = input_image_centroids_arranged[i].first;

        ret.push_back(Eigen::Vector2d(p[0],p[1]));
        cv::circle(input_image_centroids_dbg, cv::Vec2i(p),20,cv::Scalar (255,0,0),5);
        cv::putText(input_image_centroids_dbg , std::to_string(i), cv::Vec2i(p)+cv::Vec2i(10,10), 5,5,cv::Scalar (255,255,0));
    }

    cv::imwrite("/tmp/dbg_10_input_image_centroids.png",input_image_centroids_dbg);
    return ret;
}



//Eigen::Vector2d m3d_utils::projectPoint(const Eigen::Vector4d& p, const sensor_msgs::CameraInfo & ci)
//{
//    const double &fx = ci.K.elems[0];
//    const double &cx = ci.K.elems[2];
//    const double &fy = ci.K.elems[4];
//    const double &cy = ci.K.elems[5];
//
//    const float x = fx*p.x()/p.z() + cx;
//    const float y = fy*p.y()/p.z() + cy;
//    // distort point
//    return Eigen::Vector2d{x, y};
//};

Eigen::Matrix3d m3d_utils::findCovariance ( std::vector<Eigen::Vector3d> points ,const Eigen::Vector3d avg)
{
    Eigen::Matrix3d covariance;
    for (int x = 0; x < 3; x ++)
    {
        for (int y = 0; y < 3; y ++)
        {
            double element =0;
            for (const auto pp : points)
            {
                element += (pp(x) - avg(x)) * (pp(y) - avg(y));

            }
            covariance(x,y) = element / (points.size()-1);
        }
    };
    std::cout << "coviariance " << covariance<< std::endl;
    return covariance;
}

Eigen::Matrix3d m3d_utils::findCovariance ( std::vector<Eigen::Vector4d> points ,const Eigen::Vector4d avg)
{
    Eigen::Matrix3d covariance;
    for (int x = 0; x < 3; x ++)
    {
        for (int y = 0; y < 3; y ++)
        {
            double element =0;
            for (const auto pp : points)
            {
                if ((avg- pp).norm () < 2) {
                    element += (pp(x) - avg(x)) * (pp(y) - avg(y));
                }
            }
            covariance(x,y) = element / (points.size()-1);
        }
    };
    return covariance;
}



void m3d_utils::saveCFG( const std::vector<double> & laser1_config, const std::vector<double> &camera1_config){
    boost::property_tree::ptree pt;
    for (int i =0; i < laser1_config.size(); i++)
    {
        pt.put("laser1_config."+std::to_string(i), laser1_config[i]);
    }
    for (int i =0; i < camera1_config.size(); i++)
    {
        pt.put("camera1_config."+std::to_string(i), camera1_config[i]);
    }

    boost::property_tree::write_ini( "cfg.ini", pt );
}
void m3d_utils::tryLoadCFG(std::vector<double> & laser1_config, std::vector<double> &camera1_config){
    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_ini("cfg.ini", pt);
        for (int i =0; i < laser1_config.size(); i++)
        {
            laser1_config[i] = pt.get<float>("laser1_config."+std::to_string(i));
        }
        for (int i =0; i < camera1_config.size(); i++)
        {
            camera1_config[i] = pt.get<float>("camera1_config."+std::to_string(i));
        }
    }
    catch (const boost::property_tree::ptree_error &e) {
        std::cout << " error during loading INI :" << e.what() << std::endl;
    }
}

Eigen::Matrix4d m3d_utils::ladybug_camera_calibration::makeTransformation( const double rotX, const double rotY, const double rotZ, const double transX, const double transY, const double transZ )
{
    Eigen::Matrix4d matrix;
    double cosX, sinX, cosY, sinY, cosZ, sinZ;

    cosX = cos( rotX );		sinX = sin( rotX );
    cosY = cos( rotY );		sinY = sin( rotY );
    cosZ = cos( rotZ );		sinZ = sin( rotZ );

    // translation portion of transform
    matrix(0,3) = transX;
    matrix(1,3) = transY;
    matrix(2,3) = transZ;

    // cz*cy;
    matrix(0,0) = cosZ * cosY;
    // cz*sy*sx - sz*cx;
    matrix(0,1) = cosZ * sinY * sinX - sinZ * cosX;
    // cz*sy*cx + sz*sx;
    matrix(0,2) = cosZ * sinY * cosX + sinZ * sinX;

    // sz*cy;
    matrix(1,0) = sinZ * cosY;
    // sz*sy*sx + cz*cx;
    matrix(1,1) = sinZ * sinY * sinX + cosZ * cosX;
    // sz*sy*cx - cz*sx;
    matrix(1,2) = sinZ * sinY * cosX - cosZ * sinX;

    //-sy;
    matrix(2,0) = -sinY;
    //cy*sx;
    matrix(2,1) = cosY * sinX;
    //cy*cx;
    matrix(2,2) = cosY * cosX;

    // bottom row, always the same
    matrix(3,0) = 0.0;
    matrix(3,1) = 0.0;
    matrix(3,2) = 0.0;
    matrix(3,3) = 1.0;
    return matrix;
}


const std::vector<Eigen::Matrix4f> &m3d_utils::ladybug_camera_calibration::getCameraExtrinsic() const {
    return camera_extrinsic;
}

const std::vector<float> &m3d_utils::ladybug_camera_calibration::getCameraFocal() const {
    return camera_focal;
}

const std::vector<Eigen::Vector2f> &m3d_utils::ladybug_camera_calibration::getCameraCenter() const {
    return camera_center;
}


void m3d_utils::ladybug_camera_calibration::loadCfgFromString(const std::string &data){
    std::stringstream  ss(data);
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(ss, pt);
    this->camera_center.resize(6);
    this->camera_extrinsic.resize(6);
    this->camera_focal.resize(6);
    for (int i =0; i < 6; i++)
    {
        camera_center[i].x() = pt.get<float>("camera"+std::to_string(i)+"_CameraCenterX");
        camera_center[i].y() = pt.get<float>("camera"+std::to_string(i)+"_CameraCenterY");
        camera_focal[i] = pt.get<float>("camera"+std::to_string(i)+"_FocalLen");
        const float tx = pt.get<float>("camera"+std::to_string(i)+"_transX");
        const float ty = pt.get<float>("camera"+std::to_string(i)+"_transY");
        const float tz = pt.get<float>("camera"+std::to_string(i)+"_transZ");
        const float rx = pt.get<float>("camera"+std::to_string(i)+"_rotX");
        const float ry = pt.get<float>("camera"+std::to_string(i)+"_rotY");
        const float rz = pt.get<float>("camera"+std::to_string(i)+"_rotZ");
        camera_extrinsic[i] = makeTransformation(rx,ry,rz,tx,ty,tz).cast<float>();
    }
}
void m3d_utils::ladybug_camera_calibration::loadConfig(const std::string &fn){

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(fn, pt);

    this->camera_center.resize(6);
    this->camera_extrinsic.resize(6);
    this->camera_focal.resize(6);
    for (int i =0; i < 6; i++)
    {
        camera_center[i].x() = pt.get<float>("camera"+std::to_string(i)+"_CameraCenterX");
        camera_center[i].y() = pt.get<float>("camera"+std::to_string(i)+"_CameraCenterY");
        camera_focal[i] = pt.get<float>("camera"+std::to_string(i)+"_FocalLen");
        const float tx = pt.get<float>("camera"+std::to_string(i)+"_transX");
        const float ty = pt.get<float>("camera"+std::to_string(i)+"_transY");
        const float tz = pt.get<float>("camera"+std::to_string(i)+"_transZ");
        const float rx = pt.get<float>("camera"+std::to_string(i)+"_rotX");
        const float ry = pt.get<float>("camera"+std::to_string(i)+"_rotY");
        const float rz = pt.get<float>("camera"+std::to_string(i)+"_rotZ");
        camera_extrinsic[i] = makeTransformation(rx,ry,rz,tx,ty,tz).cast<float>();
    }

}
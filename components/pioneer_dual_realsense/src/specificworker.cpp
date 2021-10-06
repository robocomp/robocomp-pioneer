/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <chrono>
#include <ranges>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

    serial_left = params["device_serial_left"].value;
    serial_right = params["device_serial_right"].value;
    print_output = (params["display"].value == "true") or (params["display"].value == "True");
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    try
    {
        auto list = ctx.query_devices();
        for (int i=0;i<list.size();i++)
        {
            rs2::device dev=list[i];
            std::string dev_serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            cout << dev_serial_number;

                cout<<"Device found"<<endl;
                dev.hardware_reset();

        }
        cout<<"Device reseted"<<endl;
        cfg_left.enable_device(serial_left);
        cfg_left.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg_left.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        rs2::pipeline pipe_left(ctx);
        pipe_left.start(cfg_left);
        left_cam_intr = pipe_left.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        left_depth_intr = pipe_left.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        pipelines.emplace_back(pipe_left);



        cfg_right.enable_device(serial_right);
        cfg_right.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg_right.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        rs2::pipeline pipe_right(ctx);
        pipe_right.start(cfg_right);
        right_cam_intr = pipe_right.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        right_depth_intr = pipe_right.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        pipelines.emplace_back(pipe_right);

        filters.emplace_back("Decimate", dec_filter);
        //filters.emplace_back(disparity_filter_name, depth_to_disparity);
        filters.emplace_back("Spatial", spat_filter);
        filters.emplace_back("Temporal", temp_filter);
        filters.emplace_back("HFilling", holef_filter);
    }
    catch(std::exception &e)
    { std::cout<<e.what()<<std::endl; }

    this->Period = 50;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);

    qInfo()<<"Fin Initialize worker";
}

void SpecificWorker::compute()
{
    auto start = chrono::steady_clock::now();
    //const auto &[points, frame_list] = read_and_filter();
    //    std::future<std::tuple<std::vector<rs2::points>, std::vector<rs2::frameset>>> futRF = std::async(&SpecificWorker::read_and_filter, this);
    //
    //    std::future<std::tuple<cv::Mat, std::vector<SpecificWorker::LaserPoint>>> futMos = std::async(&SpecificWorker::mosaic, this, std::move(futRF));
    //    //std::async(&SpecificWorker::mostrar, this, std::move(futMos));
    //
    //    auto [m, this->vector_laser] = futMos.get();
    auto [points,frame_list] = read_and_filter();
    auto
    [m, vector_laser] = mosaic(points[0],points[1],frame_list[0],frame_list[1]);

    this->vector_laser = vector_laser;
    cv::resize(m,m,cv::Size(1600,900));

    if (print_output) {
        cv::imshow("Virtual", m);
        cv::waitKey(1); //??
    }

    vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(50);

    bufferMutex.lock();
    cv::imencode(".jpg", m, buffer, compression_params);
    bufferMutex.unlock();

    auto end = chrono::steady_clock::now();
    qInfo() << "Elapsed time in seconds: "
            << chrono::duration_cast<chrono::milliseconds>(end - start).count()
            << " ms";

}

std::tuple<std::vector<rs2::points>, std::vector<rs2::frameset>> SpecificWorker::read_and_filter()
{
    std::vector<rs2::frameset> frame_list(2);
    std::vector<rs2::points> points(2);
    for (auto &&[i, pipe] : iter::enumerate(pipelines))
    {
        rs2::frameset data = pipe.wait_for_frames();
        frame_list[i] = data;
        depth_list[i] = data.get_depth_frame(); // Find and colorize the depth data

        for (auto &&filter : filters)
            if (filter.is_enabled)
                depth_list[i] = filter.filter.process(depth_list[i]);

        rgb_list[i] = data.get_color_frame(); // Find the color data
        points[i] = pointclouds[i].calculate(depth_list[i]);
        pointclouds[i].map_to(rgb_list[i]);
    }
    return std::make_tuple(points, frame_list);
}

//const rs2::points points_left, const rs2::points points_right, const rs2::frameset cdata_left, const rs2::frameset cdata_right
//std::future<std::tuple<std::vector<rs2::points>, std::vector<rs2::frameset>>> mos
std::tuple<cv::Mat, std::vector<SpecificWorker::LaserPoint>> SpecificWorker::mosaic(const rs2::points points_left, const rs2::points points_right, const rs2::frameset cdata_left, const rs2::frameset cdata_right){

    rs2::video_frame left_image = cdata_left.get_color_frame();
    rs2::video_frame right_image = cdata_right.get_color_frame();
    cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(left_cam_intr.width * 2.5, left_cam_intr.height * 1.5),
                                           CV_8UC3);
    float center_virtual_cols = frame_virtual.cols / 2.0;
    float center_virtual_rows = frame_virtual.rows / 2.0;
    float frame_virtual_lfocalx = left_cam_intr.fx;
    float frame_virtual_rfocalx = right_cam_intr.fx;

    // laser stuff
    const int MAX_LASER_BINS = 100;
    const float TOTAL_HOR_ANGLE = 2.094;  // para 120º
    using Point = std::tuple<float, float, float>;
    auto cmp = [](Point a, Point b) {
        auto &[ax, ay, az] = a;
        auto &[bx, by, bz] = b;
        return (ax * ax + ay * ay + az * az) < (bx * bx + by * by + bz * bz);
    };
    std::vector<std::set<Point, decltype(cmp) >> hor_bins(MAX_LASER_BINS);
    rs2::video_frame left_depth = cdata_left.get_depth_frame();
    if (left_cam_intr.width == left_depth_intr.width and left_cam_intr.height == left_depth_intr.height) {
        float coseno = cos(-consts.rotated_angle);
        float seno = sin(-consts.rotated_angle);
        float h_offset = consts.semi_distance_to_center; //0.3
        const rs2::vertex *vertices = points_left.get_vertices();
        auto tex_coords = points_left.get_texture_coordinates(); // and texture coordinates, u v coor of rgb image
        for (size_t i = 0; i < points_left.size(); i++)
        {
            if (vertices[i].z)
            {
                // Y downwards and Z outwards
                // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
                float XV = coseno * vertices[i].x - seno * vertices[i].z + h_offset;
                float ZV = seno * vertices[i].x + coseno * vertices[i].z;
                float YV = vertices[i].y ;
                // project
                int col_virtual = static_cast<int>(fabs(frame_virtual_lfocalx * XV / ZV + center_virtual_cols));
                int row_virtual = static_cast<int>(fabs(
                        frame_virtual_lfocalx * vertices[i].y / ZV + center_virtual_rows));

                if (col_virtual >= frame_virtual.cols or row_virtual >= frame_virtual.rows) continue;

                //col_virtual += center_virtual_j/2;
                //qInfo() << "coor " << vertices[i].x << vertices[i].y << vertices[i].z << col_virtual << row_virtual;

                int k = tex_coords[i].v * left_image.get_height();
                int l = tex_coords[i].u * left_image.get_width();

                if (k < 0 or k >= left_image.get_height() or l < 0 or l > left_image.get_width()) continue;

                color(left_image, frame_virtual, row_virtual, col_virtual, k, l);

                // laser computation
                if (YV < consts.laser_down_cut_threshold) continue;
                // accumulate in bins of equal horizontal angle from optical axis
                float hor_angle = atan2(XV, ZV);
                // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                int angle_index = (int) ((MAX_LASER_BINS / TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS / 2));
                if (angle_index >= 100 or angle_index < 0) continue;
                hor_bins[angle_index].emplace(std::make_tuple(XV, YV, ZV));
            }
        }
    }

    rs2::video_frame right_depth = cdata_right.get_depth_frame();
    if (right_cam_intr.width == right_depth_intr.width and right_cam_intr.height == right_depth_intr.height) {
        float coseno = cos(consts.rotated_angle);
        float seno = sin(consts.rotated_angle);
        float h_offset = consts.semi_distance_to_center;
        const rs2::vertex *vertices = points_right.get_vertices();
        auto tex_coords = points_right.get_texture_coordinates(); // and texture coordinates, u v coor of rgb image

        for (size_t i = 0; i < points_right.size(); i++)
        {
            if (vertices[i].z)
            {
                // (Y outwards and Z up)
                // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
                float XV = coseno * vertices[i].x - seno * vertices[i].z + h_offset;
                float ZV = seno * vertices[i].x + coseno * vertices[i].z;
                float YV = vertices[i].y;

                // project
                float col_virtual = static_cast<int>(fabs(frame_virtual_rfocalx * XV / ZV + center_virtual_cols));
                float row_virtual = static_cast<int>(fabs(frame_virtual_rfocalx * vertices[i].y / ZV + center_virtual_rows));
                //qInfo() << "coor " << vertices[i].x << vertices[i].y << vertices[i].z << col_virtual << row_virtual;
                if (col_virtual >= frame_virtual.cols or row_virtual >= frame_virtual.rows) continue;

                int k = tex_coords[i].v * right_image.get_height();
                int l = tex_coords[i].u * right_image.get_width();
                if (k < 0 or k >= right_image.get_height() or l < 0 or l > right_image.get_width()) continue;

                color(right_image, frame_virtual, row_virtual, col_virtual, k, l);

                // laser computation
                if (YV < consts.laser_down_cut_threshold) continue;
                // accumulate in bins of equal horizontal angle from optical axis
                float hor_angle = atan2(XV, ZV) ;
                // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                int angle_index = (int) ((MAX_LASER_BINS / TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS / 2));
                if (angle_index >= 100 or angle_index < 0) continue;
                hor_bins[angle_index].emplace(std::make_tuple(XV, YV, ZV));
            }
        }
    }

    //tratamiento imagen
    cv::GaussianBlur(frame_virtual, frame_virtual, cv::Size(13, 13), 0, 0, 0);
    float alpha = 4.0;
    float beta = -1.0;
    frame_virtual.convertTo(frame_virtual, -1, alpha, beta);

    // laser computation
    std::vector<LaserPoint> laser_data(MAX_LASER_BINS);
    uint i = 0;
    for (auto &bin : hor_bins)
    {
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            laser_data[i] = LaserPoint{sqrt(X * X + Y * Y + Z * Z),
                                       (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        } else
        {
            laser_data[i] = LaserPoint{0.f, (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        }
        i++;
    }
//    auto laser_poly = filter_laser(laser_data);
//    draw_laser(&scene, laser_poly);

    return std::make_tuple(frame_virtual, laser_data);

}


std::tuple<cv::Mat, std::vector<SpecificWorker::LaserPoint>> SpecificWorker::mosaicDOS(const rs2::points points_left, const rs2::points points_right, const rs2::frameset cdata_left, const rs2::frameset cdata_right){

    rs2::video_frame left_image = cdata_left.get_color_frame();
    rs2::video_frame right_image = cdata_right.get_color_frame();
    cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(left_cam_intr.width * 1.5, left_cam_intr.height * 1.5),
                                           CV_8UC3);
    float center_virtual_cols = frame_virtual.cols / 2.0;
    float center_virtual_rows = frame_virtual.rows / 2.0;
    float frame_virtual_lfocalx = left_cam_intr.fx;
    float frame_virtual_rfocalx = right_cam_intr.fx;

    // laser stuff
    const int MAX_LASER_BINS = 100;
    const float TOTAL_HOR_ANGLE = 1;  // para 60º
    using Point = std::tuple<float, float, float>;
    auto cmp = [](Point a, Point b) {
        auto &[ax, ay, az] = a;
        auto &[bx, by, bz] = b;
        return (ax * ax + ay * ay + az * az) < (bx * bx + by * by + bz * bz);
    };
    std::vector<std::set<Point, decltype(cmp) >> hor_bins(MAX_LASER_BINS);
    rs2::video_frame left_depth = cdata_left.get_depth_frame();
    if (left_cam_intr.width == left_depth_intr.width and left_cam_intr.height == left_depth_intr.height)
    {
        float coseno = cos(-consts.rotated_angle);
        float seno = sin(-consts.rotated_angle);
        float h_offset = 0; //0.3
        const rs2::vertex *vertices = points_left.get_vertices();
        auto tex_coords = points_left.get_texture_coordinates(); // and texture coordinates, u v coor of rgb image
        for (size_t i = 0; i < points_left.size(); i++)
        {
            if (vertices[i].z)
            {
                // Y downwards and Z outwards
                // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
                //float XV = coseno * vertices[i].x - seno * vertices[i].z + h_offset;
                //float ZV = seno * vertices[i].x + coseno * vertices[i].z;
                float XV = vertices[i].x;
                float ZV = vertices[i].z;
                float YV = vertices[i].y ;
                // project
                int col_virtual = static_cast<int>(fabs(frame_virtual_lfocalx * XV / ZV + center_virtual_cols));
                int row_virtual = static_cast<int>(fabs(
                        frame_virtual_lfocalx * vertices[i].y / ZV + center_virtual_rows));
                if (col_virtual >= frame_virtual.cols or row_virtual >= frame_virtual.rows) continue;

                int k = tex_coords[i].v * left_image.get_height();
                int l = tex_coords[i].u * left_image.get_width();

                if (k < 0 or k >= left_image.get_height() or l < 0 or l > left_image.get_width()) continue;

                color(left_image, frame_virtual, row_virtual, col_virtual, k, l);

                // laser computation
                //if (YV < consts.laser_down_cut_threshold) continue;
                // accumulate in bins of equal horizontal angle from optical axis
                float hor_angle = atan2(XV, ZV);
                // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                int angle_index = (int) ((MAX_LASER_BINS / TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS / 2));
                if (angle_index >= 100 or angle_index < 0) continue;
                hor_bins[angle_index].emplace(std::make_tuple(XV, YV, ZV));
            }
        }
    }

    //tratamiento imagen
    cv::GaussianBlur(frame_virtual, frame_virtual, cv::Size(13, 13), 0, 0, 0);
    float alpha = 4.0;
    float beta = -1.0;
    frame_virtual.convertTo(frame_virtual, -1, alpha, beta);

    // laser computation
    std::vector<LaserPoint> laser_data(MAX_LASER_BINS);
    uint i = 0;
    for (auto &bin : hor_bins)
    {
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            laser_data[i] = LaserPoint{sqrt(X * X + Y * Y + Z * Z) * 1000,
                                       (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        } else
        {
            laser_data[i] = LaserPoint{0.f, (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        }
        i++;
    }
    qInfo() << __FUNCTION__ << "Min: " << std::ranges::min(laser_data, [](auto &a, auto &b){ return a.dist < b.dist;}).dist;
    qInfo() << __FUNCTION__ << "Max: " << std::ranges::max(laser_data, [](auto &a, auto &b){ return a.dist < b.dist;}).dist;

    return std::make_tuple(frame_virtual, laser_data);
}

void SpecificWorker::color(rs2::video_frame image, cv::Mat frame_v, int row_v, int col_v, int k, int l)
{
    auto ptr = (uint8_t *) image.get_data();
    auto stride = image.get_stride_in_bytes();

    cv::Vec3b &color = frame_v.at<cv::Vec3b>(floor(row_v), floor(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
    color = frame_v.at<cv::Vec3b>(ceil(row_v), floor(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
    color = frame_v.at<cv::Vec3b>(floor(row_v), ceil(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
    color = frame_v.at<cv::Vec3b>(ceil(row_v), ceil(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
}

/////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    RoboCompCameraRGBDSimple::TRGBD res;

    res.image.cameraID=0;
    res.image.image = m;
    res.image.depth = m.depth();
    res.image.height = m.rows;
    res.image.width = m.cols;
    res.image.focalx = left_cam_intr.fx;
    res.image.focaly = left_cam_intr.fy;
    res.image.alivetime = 9999999999999;

    return res;
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
//implementCODE
    RoboCompCameraRGBDSimple::TDepth dp;

//    dp.depth;
//    dp.cameraID = 0;
//    dp.width = left_depth_intr.width + right_depth_intr.width;
//    dp.height = left_depth_intr.height;
//    dp.focalx = left_depth_intr.fx;
//    dp.focaly = left_depth_intr.fy;
//    dp.alivetime;
//    dp.depthFactor;

    return dp;
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    std::lock_guard<std::mutex> ml(bufferMutex);
    RoboCompCameraRGBDSimple::TImage im;
    im.cameraID=0;
    im.image = buffer;
    im.depth = m.depth();
    im.height = m.rows;
    im.width = m.cols;
    im.focalx = left_cam_intr.fx;
    im.focaly = left_cam_intr.fy;
    im.alivetime = chrono::duration_cast<chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return im;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
//implementCODE

}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
//implementCODE
    RoboCompLaser::TLaserData l;

    //std::transform(this->vector_laser.begin(),this->vector_laser.end(),std::back_inserter(l),[](const auto ldata){return RoboCompLaser::TData{ldata.dist,ldata.angle};});
    for(auto &v: this->vector_laser){
        l.emplace_back(RoboCompLaser::TData{v.angle, v.dist});
    }
    // Aparece el cero
    //qInfo() << __FUNCTION__ << __LINE__<< &this->vector_laser << this->vector_laser.size();
    return l;
}

RoboCompMonitorBase::MonitorStates SpecificWorker::MonitorBase_getMonitorState()
{
//implementCODE
}




//cv::Mat SpecificWorker::mosaic( const rs2::frameset &cdata_left, const rs2::frameset &cdata_right, unsigned short subsampling )
//{
//    // check that both cameras are equal
//    // declare frame virtual
//    rs2::video_frame left_image = cdata_left.get_color_frame();
//    rs2::video_frame right_image = cdata_right.get_color_frame();
//    cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(left_cam_intr.width*3, left_cam_intr.height), CV_8UC3);
//    float center_virtual_i = frame_virtual.cols / 2.0;
//    float center_virtual_j = frame_virtual.rows / 2.0;
//    float frame_virtual_focalx = left_cam_intr.fx;
//    //auto before = myclock::now();
//
//    // laser stuff
//    const int MAX_LASER_BINS = 100;
//    const float TOTAL_HOR_ANGLE = 2.094;  // para 120º
//    using Point = std::tuple< float, float, float>;
//    auto cmp = [](Point a, Point b) { auto &[ax,ay,az] = a; auto &[bx,by,bz] = b; return (ax*ax+ay*ay+az*az) < (bx*bx+by*by+bz*bz);};
//    std::vector<std::set<Point, decltype(cmp) >> hor_bins(MAX_LASER_BINS);
//
//    // Left image check that rgb and depth are equal
//    rs2::video_frame left_depth = cdata_left.get_depth_frame();
//    if(left_cam_intr.width == left_depth_intr.width and left_cam_intr.height == left_depth_intr.height)
//    {
//        // cast depth
//        float *depth_array = (float *) left_depth.get_data();
//        // cast rgb
//        const auto &rgb_img_data =(uchar *) reinterpret_cast<const uint8_t *>(left_image.get_data());
//        // vars
//        float X, Y, Z;
//        int cols, rows;
//        std::size_t num_pixels = left_depth_intr.width * left_depth_intr.height;
//        float coseno = cos(-M_PI / 6.0);
//        float seno = sin(-M_PI / 6.0);
//        float h_offset = -100;
//        for (std::size_t i = 0; i < num_pixels; i += subsampling)
//        {
//            cols = (i % left_depth_intr.width) - (left_depth_intr.width / 2);
//            rows = (left_depth_intr.height / 2) - (i / left_depth_intr.height);
//            // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
//            Y = depth_array[i] * 1000.f; // we transform measurements to millimeters
//            if (Y < 100) continue;
//            X = -cols * Y / left_depth_intr.fx;
//            Z = rows * Y / left_depth_intr.fx;
//            // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
//            float XV = coseno * X - seno * Y + h_offset;
//            float YV = seno * X + coseno * Y;
//            // project on virtual camera
//            auto col_virtual = frame_virtual_focalx * XV / YV + center_virtual_i;
//            auto row_virtual = frame_virtual_focalx * Z / YV + center_virtual_j;
//            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), floor(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), ceil(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), floor(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), ceil(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            // laser computation
//            if(Z>50 or Z<-450) continue;  // above the robot and on the floor
//            // accumulate in bins of equal horizontal angle from optical axis
//            float hor_angle = atan2(cols, left_depth_intr.fx);
//            // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
//            int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
//            hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
//        }
//    }
//    else
//    {
//        qWarning() << __FUNCTION__ << " Depth and RGB sizes not equal";
//        return cv::Mat();
//    }
//    // right image
//    rs2::video_frame right_depth = cdata_right.get_depth_frame();
//    if(right_cam_intr.width == right_depth_intr.width and right_cam_intr.height == right_depth_intr.height)
//    {
//        // cast depth
//        float *depth_array = (float *) right_depth.get_data();
//        // cast rgb
//        const auto &rgb_img_data =(uchar *) reinterpret_cast<const uint8_t *>(right_image.get_data());
//        // vars
//        float X, Y, Z;
//        int cols, rows;
//        std::size_t num_pixels = right_depth_intr.width * right_depth_intr.height;
//        float coseno = cos(M_PI / 6.0);
//        float seno = sin(M_PI / 6.0);
//        float h_offset = 100;
//        for (std::size_t i = 0; i < num_pixels; i += subsampling)
//        {
//            cols = (i % right_depth_intr.width) - (right_depth_intr.width / 2);
//            rows = (right_depth_intr.height / 2) - (i / right_depth_intr.height);
//            // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
//            Y = depth_array[i] * 1000.f; // we transform measurements to millimeters
//            if (Y < 100) continue;
//            X = -cols * Y / right_depth_intr.fx;
//            Z = rows * Y / right_depth_intr.fx;
//            // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
//            float XV = coseno * X - seno * Y + h_offset;
//            float YV = seno * X + coseno * Y;
//            // project on virtual camera
//            auto col_virtual = frame_virtual_focalx * XV / YV + center_virtual_i;
//            auto row_virtual = frame_virtual_focalx * Z / YV + center_virtual_j;
//            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), floor(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), ceil(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), floor(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
//            {
//                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), ceil(col_virtual));
//                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
//            }
//            // laser computation
//            if(Z>50) continue;
//            // accumulate in bins of equal horizontal angle from optical axis
//            float hor_angle = atan2(cols, right_depth_intr.fx);
//            // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
//            int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
//            hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
//        }
//    }
//    else
//    {
//        qWarning() << __FUNCTION__ << " Depth and RGB sizes not equal";
//        return cv::Mat();
//    }
//    // Fill gaps
//    //cv::inpaint(frame_virtual, frame_virtual_occupied, frame_virtual, 1.0, cv::INPAINT_TELEA);
//    cv::medianBlur(frame_virtual, frame_virtual, 3);
//    //msec duration = myclock::now() - before;
//    //std::cout << "It took " << duration.count() << "ms" << std::endl;
//    //before = myclock::now();   // so it is remembered across QTimer calls to compute()
//
//    //qInfo() << frame_virtual.step[0] * frame_virtual.rows;;
//    //vector<int> compression_params;
//    //compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
//    //compression_params.push_back(50);
//    //vector<uchar> buffer;
//    //cv::imencode(".jpg", frame_virtual, buffer, compression_params);
//    //duration = myclock::now() - before;
//    //std::cout << "Encode took " << duration.count() << "ms" << std::endl;
//    //qInfo() << "comp " << buffer.size();
//
//    cv::flip(frame_virtual, frame_virtual, -1);
//   // cv::Mat frame_virtual_final(label_rgb->width(),label_rgb->height(), CV_8UC3);
//   // cv::resize(frame_virtual, frame_virtual_final, cv::Size(label_rgb->width(),label_rgb->height()), 0, 0, cv::INTER_LANCZOS4);
//
//    // laser computation
//   /* std::vector<LaserPoint> laser_data(MAX_LASER_BINS);
//    uint i=0;
//    for(auto &bin : hor_bins)
//    {
//        if( bin.size() > 0)
//        {
//            const auto &[X, Y, Z] = *bin.cbegin();
//            laser_data[i] = LaserPoint{sqrt(X * X + Y * Y + Z * Z), (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
//        }
//        else
//            laser_data[i] = LaserPoint{0.f,(i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
//        i++;
//    }*/
//    //auto laser_poly = filter_laser(laser_data);
//    //draw_laser(&scene, laser_poly);
//
//    return frame_virtual;
//}



////////////////////////////////////////////////////////////////////////////////////



/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData
